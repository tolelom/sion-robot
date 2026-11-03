"""경로 추종: 웨이포인트를 TURN → DRIVE 구간으로 나눠 개루프로 주행한다.

위치는 명령한 선속도·각속도를 적분하는 추측 항법(PoseEstimator)으로만 추정한다.
"""
import logging
import math
import threading
import time
from typing import Callable, List, Optional, Tuple

from config import (
    OMEGA_TURN, SCALE_OMEGA_RAD, SCALE_V_MPS, SEGMENT_PAUSE_SEC, SERVO_ATTACK_HIT,
    SERVO_ATTACK_HIT_TIME, SERVO_ATTACK_PAUSE_SEC, SERVO_ATTACK_PREP, SERVO_ATTACK_TURN_DEG,
    SERVO_INIT_ANGLES, SERVO_SPEED, SERVO_TIME, STEP_SEC, V_CHARGE, V_CRUISE,
)
from hardware import RobotBase

logger = logging.getLogger(__name__)

Pose = Tuple[float, float, float]  # (x[m], y[m], theta[rad])
PoseCallback = Callable[[float, float, float], None]

# 목표 방향과의 차이가 이보다 작으면 TURN을 생략한다.
HEADING_TOLERANCE_RAD = math.radians(1.0)
# 길이가 이보다 짧은 구간은 같은 점으로 보고 건너뛴다.
MIN_SEGMENT_LEN_M = 1e-6
# 속도로 나눌 때 0 나눗셈을 막는 하한.
_MIN_RATE = 1e-6

# 적 액션 회전 방향: 항상 양(반시계). 하드웨어 배치 가정.
_ENEMY_ATTACK_TURN_DIR = 1.0


def normalize_angle(theta: float) -> float:
    """각도를 [-pi, pi) 범위로 접는다."""
    return (theta + math.pi) % (2 * math.pi) - math.pi


# ==========================
# Pose Estimator (open-loop)
# ==========================

class PoseEstimator:
    def __init__(self, x0: float, y0: float, theta0: float = 0.0):
        self.x = x0
        self.y = y0
        self.theta = theta0  # rad

    def update(self, v_mps: float, omega_rad: float, dt: float) -> None:
        self.x += v_mps * math.cos(self.theta) * dt
        self.y += v_mps * math.sin(self.theta) * dt
        self.theta = normalize_angle(self.theta + omega_rad * dt)

    def get_pose(self) -> Pose:
        return self.x, self.y, self.theta


# ==========================
# 모션 실행기
# ==========================

class _MotionRunner:
    """모터 명령을 일정 시간 유지하면서 포즈를 적분하고 콜백으로 알린다.

    cancel_event가 set되면 가능한 빨리 멈춘다. 모든 구간은 robot.stop()으로 끝난다.
    """

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
        # pose_update_cb는 외부 입력. 거기서 던진 예외가 모터 정지 흐름까지 깨지 않게 격리.
        if self._pose_update_cb is None:
            return
        try:
            self._pose_update_cb(*self.pose_est.get_pose())
        except Exception:
            logger.exception("pose_update_cb raised")

    def sleep(self, sec: float) -> bool:
        """sec초 대기. cancel되면 즉시 True를 반환한다."""
        if self._cancel_event is not None:
            return self._cancel_event.wait(timeout=sec)
        # cancel_event가 없으면 time.sleep이 더 정확하다(wait는 ms 정확도가 떨어질 수 있다).
        time.sleep(sec)
        return False

    def run_for_duration(self, v_norm: float, omega_norm: float,
                         v_mps: float, omega_rad: float, duration: float) -> None:
        """duration 동안 step_sec 간격으로 모터 명령 유지 + 포즈 적분 + 콜백 호출.

        취소되지 않으면 적분 구간의 합은 정확히 duration이 된다(마지막 조각은 t_end까지).
        취소되면 마지막으로 적분한 시점까지만 반영한다 — 곧 모터가 정지.
        """
        self.robot.set_velocity(v_norm, omega_norm)
        start = time.monotonic()
        last_t = start
        t_end = start + duration

        while not self.is_cancelled():
            now = time.monotonic()
            if now >= t_end:
                # 마지막 조각: last_t ~ t_end. 이걸 빼먹으면 구간마다 최대 step_sec만큼 덜 움직인 것으로 추정된다.
                if t_end > last_t:
                    self.pose_est.update(v_mps, omega_rad, t_end - last_t)
                    self.report_pose()
                break

            dt = now - last_t
            last_t = now
            if dt > 0:
                self.pose_est.update(v_mps, omega_rad, dt)
                self.report_pose()

            sleep_t = max(0.0, self.step_sec - (time.monotonic() - now))
            if sleep_t > 0 and self.sleep(sleep_t):
                break

        self.robot.stop()

    def turn(self, angle_rad: float, omega_turn_norm: float, scale_omega_rad: float) -> float:
        """제자리에서 angle_rad만큼 회전한다(부호 = 방향). 회전 시간(초)을 반환한다."""
        direction = 1.0 if angle_rad > 0 else -1.0
        omega_real = omega_turn_norm * scale_omega_rad  # rad/s
        turn_time = abs(angle_rad) / max(omega_real, _MIN_RATE)
        self.run_for_duration(
            v_norm=0.0,
            omega_norm=direction * omega_turn_norm,
            v_mps=0.0,
            omega_rad=direction * omega_real,
            duration=turn_time,
        )
        return turn_time

    def drive(self, length_m: float, v_norm: float, scale_v_mps: float) -> float:
        """현재 방향으로 length_m만큼 직진한다. 주행 시간(초)을 반환한다."""
        v_real = v_norm * scale_v_mps
        drive_time = length_m / max(v_real, _MIN_RATE)
        self.run_for_duration(
            v_norm=v_norm,
            omega_norm=0.0,
            v_mps=v_real,
            omega_rad=0.0,
            duration=drive_time,
        )
        return drive_time


# ==========================
# 적 목표 도달 시 서보 액션
# ==========================

def _execute_enemy_action(runner: _MotionRunner, omega_turn_norm: float, scale_omega_rad: float) -> None:
    """적 목표 도달 시 서보 액션 실행: 160도 회전 + 팔 동작"""
    robot = runner.robot
    for sid, (angle, t) in SERVO_ATTACK_PREP.items():
        robot.set_servo(sid, angle, SERVO_SPEED, t)

    runner.turn(_ENEMY_ATTACK_TURN_DIR * math.radians(SERVO_ATTACK_TURN_DEG),
                omega_turn_norm, scale_omega_rad)
    if runner.is_cancelled():
        logger.info("enemy action cancelled before HIT")
        return

    for sid, angle in SERVO_ATTACK_HIT.items():
        robot.set_servo(sid, angle, SERVO_SPEED, SERVO_ATTACK_HIT_TIME)
    # 대기 중 취소되어도 팔은 초기 자세로 돌려놓는다(대기만 건너뛴다).
    cancelled = runner.sleep(SERVO_ATTACK_PAUSE_SEC)

    for sid, angle in SERVO_INIT_ANGLES.items():
        robot.set_servo(sid, angle, SERVO_SPEED, SERVO_TIME)
    if cancelled or runner.sleep(SERVO_ATTACK_PAUSE_SEC):
        logger.info("enemy action cancelled after HIT (arm reset sent)")
        return

    logger.info("enemy action complete")


# ==========================
# 경로 따라가기 (상수 속도, 실시간 pose 갱신)
# ==========================

def follow_path_constant_speed(
    robot: RobotBase,
    waypoints_world: List[Tuple[float, float]],
    is_enemy_goal: bool,
    v_cruise_norm: float = V_CRUISE,
    v_charge_norm: float = V_CHARGE,
    omega_turn_norm: float = OMEGA_TURN,
    scale_v_mps: float = SCALE_V_MPS,
    scale_omega_rad: float = SCALE_OMEGA_RAD,
    pose_update_cb: Optional[PoseCallback] = None,
    step_sec: float = STEP_SEC,
    cancel_event: Optional[threading.Event] = None,
    theta0: float = 0.0,
) -> Pose:
    """웨이포인트를 따라 구간마다 TURN → DRIVE를 실행하고 최종 추정 포즈를 반환한다.

    - theta0: 출발 시 로봇이 바라보는 방향(rad). 직전 이동이 끝난 방향을 넘겨야
      첫 TURN 각도가 맞는다.
    - TURN / DRIVE 구간을 step_sec 단위로 쪼개 모터 명령을 유지하며 포즈를 적분하고
      pose_update_cb를 계속 호출한다.
    - 적 목표면 마지막 구간을 돌진 속도(v_charge_norm)로 달린 뒤 공격 동작을 한다.
    - cancel_event가 set되면 가능한 한 빠르게 정지하고 반환한다.
      finally의 robot.stop()이 모터 정지를 보장한다.
    """
    if scale_v_mps <= 0 or scale_omega_rad <= 0 or step_sec <= 0:
        raise ValueError(
            f"invalid params: scale_v_mps={scale_v_mps}, scale_omega_rad={scale_omega_rad}, step_sec={step_sec}"
        )

    if len(waypoints_world) < 2:
        logger.warning("Waypoints too short")
        x0, y0 = waypoints_world[0] if waypoints_world else (0.0, 0.0)
        return (x0, y0, theta0)

    # 초기 pose를 첫 웨이포인트 중심으로 설정
    x0, y0 = waypoints_world[0]
    pose_est = PoseEstimator(x0, y0, theta0=theta0)
    runner = _MotionRunner(robot, pose_est, step_sec, pose_update_cb, cancel_event)
    # 명령상 방향. TURN이 끝나면 목표 방향에 정렬됐다고 간주한다.
    theta_cmd = theta0

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

            # 1) TURN
            target_heading = math.atan2(dy, dx)
            angle_diff = normalize_angle(target_heading - theta_cmd)
            if abs(angle_diff) > HEADING_TOLERANCE_RAD:
                turn_time = runner.turn(angle_diff, omega_turn_norm, scale_omega_rad)
                logger.debug("Segment %d: TURN angle=%.1fdeg, time=%.2fs",
                             i, math.degrees(angle_diff), turn_time)
                theta_cmd = target_heading
            else:
                logger.debug("Segment %d: TURN skipped", i)

            # 2) DRIVE
            v_cmd = v_charge_norm if is_charge_segment else v_cruise_norm
            drive_time = runner.drive(seg_len, v_cmd, scale_v_mps)
            logger.debug("Segment %d: DRIVE len=%.3fm, v=%.2f, time=%.2fs", i, seg_len, v_cmd, drive_time)

            runner.sleep(SEGMENT_PAUSE_SEC)

            if is_charge_segment and not runner.is_cancelled():
                _execute_enemy_action(runner, omega_turn_norm, scale_omega_rad)

    finally:
        robot.stop()

    x, y, th = pose_est.get_pose()
    logger.info("Finished. est pose=(%.2f,%.2f,theta_deg=%.1f)", x, y, math.degrees(th))
    return (x, y, th)
