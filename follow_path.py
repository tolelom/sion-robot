import logging
import math
import threading
import time
from typing import List, Tuple, Optional, Callable

from hardware import RobotBase
from config import (
    V_CRUISE, V_CHARGE, OMEGA_TURN, SCALE_V_MPS, SCALE_OMEGA_RAD,
    STEP_SEC, SEGMENT_PAUSE_SEC,
    SERVO_SPEED, SERVO_TIME, SERVO_INIT_ANGLES,
    SERVO_ATTACK_TURN_DEG, SERVO_ATTACK_PREP,
    SERVO_ATTACK_HIT, SERVO_ATTACK_HIT_TIME, SERVO_ATTACK_PAUSE_SEC,
)

logger = logging.getLogger(__name__)


# ==========================
# Pose Estimator (open-loop)
# ==========================

class PoseEstimator:
    def __init__(self, x0: float, y0: float, theta0: float = 0.0):
        self.x = x0
        self.y = y0
        self.theta = theta0  # rad

    def update(self, v_mps: float, omega_rad: float, dt: float):
        self.x += v_mps * math.cos(self.theta) * dt
        self.y += v_mps * math.sin(self.theta) * dt
        self.theta += omega_rad * dt
        self.theta = (self.theta + math.pi) % (2 * math.pi) - math.pi

    def get_pose(self) -> Tuple[float, float, float]:
        return self.x, self.y, self.theta


# ==========================
# 적 목표 도달 시 서보 액션
# ==========================

_ENEMY_ATTACK_TURN_DIR = 1.0  # 적 액션 회전 방향: 항상 양(반시계). 하드웨어 배치 가정.


def _execute_enemy_action(robot: RobotBase, omega_turn_norm: float,
                          scale_omega_rad: float, run_for_duration,
                          is_cancelled: Callable[[], bool]) -> None:
    """적 목표 도달 시 서보 액션 실행: 160도 회전 + 팔 동작"""
    for sid, (angle, t) in SERVO_ATTACK_PREP.items():
        robot.set_servo(sid, angle, SERVO_SPEED, t)

    omega_real = omega_turn_norm * scale_omega_rad
    turn_time = abs(math.radians(SERVO_ATTACK_TURN_DEG)) / max(omega_real, 1e-6)

    run_for_duration(
        v_norm=0.0,
        omega_norm=_ENEMY_ATTACK_TURN_DIR * omega_turn_norm,
        v_mps=0.0,
        omega_rad=_ENEMY_ATTACK_TURN_DIR * omega_real,
        duration=turn_time,
    )
    if is_cancelled():
        logger.info("enemy action cancelled before HIT")
        return

    for sid, angle in SERVO_ATTACK_HIT.items():
        robot.set_servo(sid, angle, SERVO_SPEED, SERVO_ATTACK_HIT_TIME)
    time.sleep(SERVO_ATTACK_PAUSE_SEC)

    for sid, angle in SERVO_INIT_ANGLES.items():
        robot.set_servo(sid, angle, SERVO_SPEED, SERVO_TIME)
    time.sleep(SERVO_ATTACK_PAUSE_SEC)

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
    pose_update_cb: Optional[Callable[[float, float, float], None]] = None,
    step_sec: float = STEP_SEC,
    cancel_event: Optional[threading.Event] = None,
) -> Tuple[float, float, float]:
    """
    - TURN / DRIVE 구간을 step_sec 단위로 쪼개서:
      * 모터 명령 유지
      * pose_est.update(...) 지속 호출
      * pose_update_cb(...) 지속 호출
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
        return (x0, y0, 0.0)

    # 초기 pose를 첫 웨이포인트 중심으로 설정
    x0, y0 = waypoints_world[0]
    pose_est = PoseEstimator(x0, y0, theta0=0.0)
    theta_est = 0.0

    def safe_cb() -> None:
        # pose_update_cb는 외부 입력. 거기서 던진 예외가 모터 정지 흐름까지 깨지 않게 격리.
        if pose_update_cb is None:
            return
        try:
            pose_update_cb(*pose_est.get_pose())
        except Exception:
            logger.exception("pose_update_cb raised")

    def is_cancelled() -> bool:
        return cancel_event is not None and cancel_event.is_set()

    safe_cb()

    num_segments = len(waypoints_world) - 1

    def run_for_duration(v_norm: float, omega_norm: float, v_mps: float, omega_rad: float, duration: float):
        """
        duration 동안 step_sec 간격으로:
        - 모터 명령 유지
        - pose 적분 (첫 step 동안은 명령 전파 지연을 인정해 적분하지 않음)
        - 콜백 호출
        """
        robot.set_velocity(v_norm, omega_norm)
        start = time.monotonic()
        last_t = start
        t_end = start + duration

        while True:
            if is_cancelled():
                break
            now = time.monotonic()
            if now >= t_end:
                break

            dt = now - last_t
            last_t = now
            if dt > 0:
                pose_est.update(v_mps, omega_rad, dt)
                safe_cb()
            # dt == 0인 첫 iteration은 pose update를 건너뛰고 step_sec 슬립으로 진입.

            sleep_t = max(0.0, step_sec - (time.monotonic() - now))
            if sleep_t > 0:
                # cancel 응답성을 위해 sleep도 cancel_event.wait로 대체 가능하지만,
                # cancel_event가 None인 일반 경로에선 time.sleep이 단순. wait를 쓰면 ms 정확도가 떨어질 수 있어 분기 유지.
                if cancel_event is not None:
                    if cancel_event.wait(timeout=sleep_t):
                        break
                else:
                    time.sleep(sleep_t)

        # 마지막 잔여시간 보정(끝 경계까지). cancel로 빠진 경우엔 보정하지 않는다 — 곧 모터가 정지.
        if not is_cancelled():
            now2 = time.monotonic()
            if now2 < t_end:
                dt = t_end - now2
                pose_est.update(v_mps, omega_rad, dt)
                safe_cb()

        robot.stop()

    try:
        for i in range(num_segments):
            if is_cancelled():
                logger.info("follow cancelled at segment %d", i)
                break
            sx, sy = waypoints_world[i]
            ex, ey = waypoints_world[i + 1]

            dx = ex - sx
            dy = ey - sy
            seg_len = math.hypot(dx, dy)
            if seg_len < 1e-6:
                continue

            target_heading = math.atan2(dy, dx)

            # ----------------------------
            # 1) TURN
            # ----------------------------
            angle_diff = (target_heading - theta_est + math.pi) % (2 * math.pi) - math.pi
            if abs(angle_diff) > math.radians(1.0):
                turn_dir = 1.0 if angle_diff > 0 else -1.0

                omega_real = omega_turn_norm * scale_omega_rad      # rad/s
                turn_time = abs(angle_diff) / max(omega_real, 1e-6)

                logger.debug("Segment %d: TURN angle=%.1fdeg, time=%.2fs", i, math.degrees(angle_diff), turn_time)

                # v=0, omega=const, 실시간 적분
                run_for_duration(
                    v_norm=0.0,
                    omega_norm=turn_dir * omega_turn_norm,
                    v_mps=0.0,
                    omega_rad=turn_dir * omega_real,
                    duration=turn_time,
                )

                theta_est = target_heading  # 목표 heading으로 정렬된 것으로 간주
            else:
                logger.debug("Segment %d: TURN skipped", i)

            # ----------------------------
            # 2) DRIVE
            # ----------------------------
            if is_enemy_goal and i == num_segments - 1:
                v_cmd = v_charge_norm
            else:
                v_cmd = v_cruise_norm

            v_real = v_cmd * scale_v_mps
            drive_time = seg_len / max(v_real, 1e-6)

            logger.debug("Segment %d: DRIVE len=%.3fm, v=%.2f, time=%.2fs", i, seg_len, v_cmd, drive_time)

            # omega=0, 직진 실시간 적분
            run_for_duration(
                v_norm=v_cmd,
                omega_norm=0.0,
                v_mps=v_real,
                omega_rad=0.0,
                duration=drive_time,
            )

            time.sleep(SEGMENT_PAUSE_SEC)

            if is_enemy_goal and i == num_segments - 1 and not is_cancelled():
                _execute_enemy_action(robot, omega_turn_norm, scale_omega_rad, run_for_duration, is_cancelled)

    finally:
        robot.stop()

    x, y, th = pose_est.get_pose()
    logger.info("Finished. est pose=(%.2f,%.2f,theta_deg=%.1f)", x, y, math.degrees(th))
    return (x, y, th)
