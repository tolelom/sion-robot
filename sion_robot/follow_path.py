"""경로 추종: 웨이포인트를 TURN → DRIVE 구간으로 나눠 개루프로 주행한다.

위치는 명령한 선속도·각속도를 적분하는 추측 항법(PoseEstimator)으로만 추정한다.
"""
import logging
import math
import threading
import time
from typing import Callable, Dict, List, Optional, Tuple

from .config import (
    MOTION_STEP_SEC, OMEGA_TURN_NORM, SCALE_OMEGA_RAD, SCALE_V_MPS, SEGMENT_PAUSE_SEC,
    SERVO_ATTACK_HIT_ANGLES, SERVO_ATTACK_HIT_TIME, SERVO_ATTACK_PAUSE_SEC, SERVO_ATTACK_PREP,
    SERVO_ATTACK_TURN_DEG, SERVO_INIT_ANGLES, SERVO_SPEED, SERVO_TIME, V_CHARGE_NORM, V_CRUISE_NORM,
)
from .hardware import RobotBase

logger = logging.getLogger(__name__)

Pose = Tuple[float, float, float]  # (x[m], y[m], theta[rad])
PoseCallback = Callable[[float, float, float], None]

HEADING_TOLERANCE_RAD = math.radians(1.0)
MIN_SEGMENT_LEN_M = 1e-6
_MIN_RATE = 1e-6

# 적 공격 회전 방향은 하드웨어 배치에 맞춰 항상 반시계(+)로 고정한다.
_ENEMY_ATTACK_TURN_DIR = 1.0


def normalize_angle(theta: float) -> float:
    """각도를 [-pi, pi) 범위로 접는다."""
    return (theta + math.pi) % (2 * math.pi) - math.pi


def needs_turn(angle_diff_rad: float) -> bool:
    return abs(angle_diff_rad) > HEADING_TOLERANCE_RAD


class PoseEstimator:
    def __init__(self, x0: float, y0: float, theta0: float = 0.0):
        self.x = x0
        self.y = y0
        self.theta = theta0

    def update(self, v_mps: float, omega_rad: float, dt: float) -> None:
        self.x += v_mps * math.cos(self.theta) * dt
        self.y += v_mps * math.sin(self.theta) * dt
        self.theta = normalize_angle(self.theta + omega_rad * dt)

    def get_pose(self) -> Pose:
        return self.x, self.y, self.theta


class _MotionRunner:
    """모터 명령을 일정 시간 유지하면서 포즈를 적분해 콜백으로 알린다."""

    def __init__(self, robot: RobotBase, pose_est: PoseEstimator, step_sec: float,
                 pose_update_cb: Optional[PoseCallback], cancel_event: Optional[threading.Event]):
        self.robot = robot
        self.pose_est = pose_est
        self.step_sec = step_sec
        self._pose_update_cb = pose_update_cb
        self._cancel_event = cancel_event

    def is_cancelled(self) -> bool:
        return self._cancel_event is not None and self._cancel_event.is_set()

    def report_pose(self) -> None:
        # 외부 콜백의 예외가 모터 정지 흐름까지 깨지 않게 격리한다.
        if self._pose_update_cb is None:
            return
        try:
            self._pose_update_cb(*self.pose_est.get_pose())
        except Exception:
            logger.exception("pose_update_cb raised")

    def sleep_unless_cancelled(self, sec: float) -> bool:
        """취소되면 즉시 깨어나 True를 반환한다."""
        if self._cancel_event is not None:
            return self._cancel_event.wait(timeout=sec)
        # Event.wait는 ms 단위 정확도가 떨어질 수 있어, 취소할 일이 없으면 time.sleep을 쓴다.
        time.sleep(sec)
        return False

    def run_for_duration(self, v_norm: float, omega_norm: float,
                         v_mps: float, omega_rad: float, duration: float) -> None:
        self.robot.set_velocity(v_norm, omega_norm)
        start = time.monotonic()
        last_t = start
        t_end = start + duration

        while not self.is_cancelled():
            now = time.monotonic()
            if now >= t_end:
                self._advance_pose(v_mps, omega_rad, t_end - last_t)
                break
            self._advance_pose(v_mps, omega_rad, now - last_t)
            last_t = now

            sleep_t = max(0.0, self.step_sec - (time.monotonic() - now))
            if sleep_t > 0 and self.sleep_unless_cancelled(sleep_t):
                break

        self.robot.stop()

    def _advance_pose(self, v_mps: float, omega_rad: float, dt: float) -> None:
        if dt > 0:
            self.pose_est.update(v_mps, omega_rad, dt)
            self.report_pose()

    def turn(self, angle_rad: float, omega_turn_norm: float, scale_omega_rad: float) -> float:
        """angle_rad의 부호가 회전 방향이다. 걸린 시간(초)을 반환한다."""
        direction = 1.0 if angle_rad > 0 else -1.0
        omega_rad = omega_turn_norm * scale_omega_rad
        turn_sec = abs(angle_rad) / max(omega_rad, _MIN_RATE)
        self.run_for_duration(
            v_norm=0.0,
            omega_norm=direction * omega_turn_norm,
            v_mps=0.0,
            omega_rad=direction * omega_rad,
            duration=turn_sec,
        )
        return turn_sec

    def drive(self, length_m: float, v_norm: float, scale_v_mps: float) -> float:
        """걸린 시간(초)을 반환한다."""
        v_mps = v_norm * scale_v_mps
        drive_sec = length_m / max(v_mps, _MIN_RATE)
        self.run_for_duration(
            v_norm=v_norm,
            omega_norm=0.0,
            v_mps=v_mps,
            omega_rad=0.0,
            duration=drive_sec,
        )
        return drive_sec


def _move_arm(robot: RobotBase, angles: Dict[int, int], servo_time: int) -> None:
    for sid, angle in angles.items():
        robot.set_servo(sid, angle, SERVO_SPEED, servo_time)


def _execute_enemy_action(runner: _MotionRunner, omega_turn_norm: float, scale_omega_rad: float) -> None:
    robot = runner.robot
    for sid, (angle, servo_time) in SERVO_ATTACK_PREP.items():
        robot.set_servo(sid, angle, SERVO_SPEED, servo_time)

    runner.turn(_ENEMY_ATTACK_TURN_DIR * math.radians(SERVO_ATTACK_TURN_DEG),
                omega_turn_norm, scale_omega_rad)
    if runner.is_cancelled():
        logger.info("enemy action cancelled before HIT")
        return

    _move_arm(robot, SERVO_ATTACK_HIT_ANGLES, SERVO_ATTACK_HIT_TIME)
    # 대기 중 취소되어도 팔은 초기 자세로 돌려놓는다(대기만 건너뛴다).
    cancelled = runner.sleep_unless_cancelled(SERVO_ATTACK_PAUSE_SEC)

    _move_arm(robot, SERVO_INIT_ANGLES, SERVO_TIME)
    if cancelled or runner.sleep_unless_cancelled(SERVO_ATTACK_PAUSE_SEC):
        logger.info("enemy action cancelled after HIT (arm reset sent)")
        return

    logger.info("enemy action complete")

def follow_path_constant_speed(
    robot: RobotBase,
    waypoints_world: List[Tuple[float, float]],
    is_enemy_goal: bool,
    v_cruise_norm: float = V_CRUISE_NORM,
    v_charge_norm: float = V_CHARGE_NORM,
    omega_turn_norm: float = OMEGA_TURN_NORM,
    scale_v_mps: float = SCALE_V_MPS,
    scale_omega_rad: float = SCALE_OMEGA_RAD,
    pose_update_cb: Optional[PoseCallback] = None,
    step_sec: float = MOTION_STEP_SEC,
    cancel_event: Optional[threading.Event] = None,
    theta0: float = 0.0,
) -> Pose:
    """최종 추정 포즈를 반환한다.

    theta0에는 직전 이동이 끝난 방향을 넘겨야 첫 TURN 각도가 맞는다.
    """
    if scale_v_mps <= 0 or scale_omega_rad <= 0 or step_sec <= 0:
        raise ValueError(
            f"invalid params: scale_v_mps={scale_v_mps}, scale_omega_rad={scale_omega_rad}, step_sec={step_sec}"
        )

    if len(waypoints_world) < 2:
        logger.warning("Waypoints too short")
        x0, y0 = waypoints_world[0] if waypoints_world else (0.0, 0.0)
        return (x0, y0, theta0)

    x0, y0 = waypoints_world[0]
    pose_est = PoseEstimator(x0, y0, theta0=theta0)
    runner = _MotionRunner(robot, pose_est, step_sec, pose_update_cb, cancel_event)
    # 개루프라 TURN을 마치면 목표 방향에 정렬됐다고 가정한다.
    assumed_heading = theta0

    runner.report_pose()

    num_segments = len(waypoints_world) - 1
    try:
        for i in range(num_segments):
            if runner.is_cancelled():
                logger.info("follow cancelled at segment %d", i)
                break
            (sx, sy), (ex, ey) = waypoints_world[i], waypoints_world[i + 1]
            dx, dy = ex - sx, ey - sy
            seg_len = math.hypot(dx, dy)
            if seg_len < MIN_SEGMENT_LEN_M:
                continue

            is_charge_segment = is_enemy_goal and i == num_segments - 1

            target_heading = math.atan2(dy, dx)
            angle_diff = normalize_angle(target_heading - assumed_heading)
            if needs_turn(angle_diff):
                turn_sec = runner.turn(angle_diff, omega_turn_norm, scale_omega_rad)
                logger.debug("Segment %d: TURN angle=%.1fdeg, time=%.2fs",
                             i, math.degrees(angle_diff), turn_sec)
                assumed_heading = target_heading
            else:
                logger.debug("Segment %d: TURN skipped", i)

            v_cmd = v_charge_norm if is_charge_segment else v_cruise_norm
            drive_sec = runner.drive(seg_len, v_cmd, scale_v_mps)
            logger.debug("Segment %d: DRIVE len=%.3fm, v=%.2f, time=%.2fs", i, seg_len, v_cmd, drive_sec)

            runner.sleep_unless_cancelled(SEGMENT_PAUSE_SEC)

            if is_charge_segment and not runner.is_cancelled():
                _execute_enemy_action(runner, omega_turn_norm, scale_omega_rad)

    finally:
        robot.stop()

    x, y, th = pose_est.get_pose()
    logger.info("Finished. est pose=(%.2f,%.2f,theta_deg=%.1f)", x, y, math.degrees(th))
    return (x, y, th)
