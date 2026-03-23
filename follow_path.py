import math
import time
from time import sleep
from typing import List, Tuple, Optional, Callable
from SCSCtrl import TTLServo

try:
    from jetbot import Robot  # JETANK 기본 제어
    
except ImportError:
    Robot = None  # PC 테스트용


# ==========================
# JETANK 모터 제어 래퍼
# ==========================

class JetankRobot:
    """
    v_norm   : -1.0 ~ 1.0
    omega_norm: -1.0 ~ 1.0  ( + : CCW )
    내부: set_motors(left, right) "포지셔널"로만 호출 (키워드 금지)
    """

    def __init__(self):
        if Robot is None:
            print("[WARN] jetbot.Robot not found. Motor control disabled (dry-run).")
            self.robot = None
        else:
            self.robot = Robot()
            # 모터의 초기 회전 임계값입니다.
            # 기어 모터는 낮은 PWM 간격에서 회전할 수 없습니다.
            # 장기간의 정전은 모터의 수명을 줄일 수 있습니다.
            moveStartTor = 0.15    

            fb_input = 0           # 전방 및 후방 방향의 속도 매개변수를 저장합니다.
            lr_input = 0           # 조향에 대한 매개변수를 저장합니다.

            servoCtrlTime = 0.001   # TTL 서보 통신 후 오류를 피하기 위한 지연입니다.
            speedInput = 1000       # 카메라의 틸트 및 팬 회전 속도입니다.

            armXStatus = 0         # 로봇 팔의 X 축 이동 상태 (원거리 및 근거리 방향)입니다.
            armYStatus = 0         # 로봇 팔의 Y 축 이동 상태 (수직 방향)입니다.
            cameStatus = 0         # 카메라의 이동 상태입니다.
            goalX = 130            # 로봇 팔의 그리퍼의 X축 좌표를 저장하세요.
            goalY = 50             # 로봇 팔의 그리퍼의 Y축 좌표를 저장하세요.
            goalC = 0              # 카메라의 위치를 저장하세요.

            xMax = 210             # 로봇 팔의 X 축 최대값을 설정하세요.
            xMin = 140             # 로봇 팔의 X 축 최소값을 설정하세요.

            yMax = 120             # 로봇 팔의 Y 축 최대값을 설정하세요.
            yMix = -120            # 로봇 팔의 Y 축 최소값을 설정하세요.

            cMax = 25              # 카메라의 최대 위치를 설정하세요.
            cMin = -45             # 카메라의 최소 위치를 설정하세요.
            cDan = 0               # 카메라의 위험한 선을 설정하세요.

            movingTime = 0.005 # 로봇 팔 역운동학 함수의 인접한 두 위치 간의 실행 시간을 설정하세요.
            movingSpeed = 1        # 로봇 팔의 이동 속도를 설정하세요.
            grabStatus = 0         # 그리퍼의 이동 상태입니다.
            cameraPosCheckMode = 1 # 카메라 위치 확인 기능의 스위치, 1 - 켜기, 0 - 끄기입니다.
                                   # 한 번 켜면, 로봇 팔과 카메라 간의 간섭을 피할 수 있습니다.
            cameraPosDanger = 0    # 카메라가 위험 지역에 있는지 여부입니다.
            cameraMoveSafe = 0     # 카메라가 안전 지역으로 이동했는지 여부입니다.
            
            TTLServo.servoAngleCtrl(1, 0, 1, 180)
            TTLServo.servoAngleCtrl(2, -80, 1, 180)
            TTLServo.servoAngleCtrl(3, -60, 1, 180)
            TTLServo.servoAngleCtrl(4, -35, 1, 180)
            print('ready!')
            

    def _set_motors_safe(self, left: float, right: float):
        left = float(max(-1.0, min(1.0, left)))
        right = float(max(-1.0, min(1.0, right)))

        if self.robot is None:
            print(f"[MOTOR] (dry-run) left={left:.2f}, right={right:.2f}")
            return

        # ✅ 포지셔널만 사용 (키워드 사용 금지!)
        self.robot.set_motors(left, right)

    def set_velocity(self, v_norm: float, omega_norm: float):
        v_norm = max(-1.0, min(1.0, v_norm))
        omega_norm = max(-1.0, min(1.0, omega_norm))

        # 차동구동 (너가 기존에 쓰던 감으로 k_omega 유지)
        k_omega = 0.7
        left = v_norm - k_omega * omega_norm
        right = v_norm + k_omega * omega_norm

        self._set_motors_safe(left, right)

    def stop(self):
        self._set_motors_safe(0.0, 0.0)


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
# 경로 따라가기 (상수 속도, 실시간 pose 갱신)
# ==========================


    

def follow_path_constant_speed(
    waypoints_world: List[Tuple[float, float]],
    is_enemy_goal: bool,
    v_cruise_norm: float = 0.35,
    v_charge_norm: float = 0.8,
    omega_turn_norm: float = 0.5,
    scale_v_mps: float = 0.15,
    scale_omega_rad: float = 0.85,
    pose_update_cb: Optional[Callable[[float, float, float], None]] = None,
    step_sec: float = 0.02,   # ✅ 20ms 단위로 계속 갱신
) -> Tuple[float, float, float]:
    """
    - TURN / DRIVE 구간을 step_sec 단위로 쪼개서:
      * 모터 명령 유지
      * pose_est.update(...) 지속 호출
      * pose_update_cb(...) 지속 호출
    """
    if len(waypoints_world) < 2:
        print("[FOLLOW] Waypoints too short.")
        x0, y0 = waypoints_world[0] if waypoints_world else (0.0, 0.0)
        return (x0, y0, 0.0)

    robot = JetankRobot()

    # 초기 pose를 첫 웨이포인트 중심으로 설정
    x0, y0 = waypoints_world[0]
    pose_est = PoseEstimator(x0, y0, theta0=0.0)
    theta_est = 0.0

    if pose_update_cb:
        pose_update_cb(*pose_est.get_pose())

    num_segments = len(waypoints_world) - 1

    def run_for_duration(v_norm: float, omega_norm: float, v_mps: float, omega_rad: float, duration: float):
        """
        duration 동안 step_sec 간격으로:
        - 모터 명령 유지
        - pose 적분
        - 콜백 호출
        """
        t_end = time.monotonic() + duration
        last_t = time.monotonic()

        # 모터 명령 시작
        robot.set_velocity(v_norm, omega_norm)

        while True:
            now = time.monotonic()
            if now >= t_end:
                break

            dt = now - last_t
            last_t = now
            if dt <= 0:
                time.sleep(step_sec)
                continue

            pose_est.update(v_mps, omega_rad, dt)
            if pose_update_cb:
                pose_update_cb(*pose_est.get_pose())

            # 일정 주기 유지
            sleep_t = max(0.0, step_sec - (time.monotonic() - now))
            if sleep_t > 0:
                time.sleep(sleep_t)

        # 마지막 잔여시간 보정(끝 경계까지)
        now2 = time.monotonic()
        if now2 < t_end:
            dt = t_end - now2
            pose_est.update(v_mps, omega_rad, dt)
            if pose_update_cb:
                pose_update_cb(*pose_est.get_pose())

        robot.stop()

    try:
        for i in range(num_segments):
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

                print(f"[FOLLOW] Segment {i}: TURN angle={math.degrees(angle_diff):.1f}deg, time={turn_time:.2f}s")

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
                print(f"[FOLLOW] Segment {i}: TURN skipped")

            # ----------------------------
            # 2) DRIVE
            # ----------------------------
            if is_enemy_goal and i == num_segments - 1:
                v_cmd = v_charge_norm
            else:
                v_cmd = v_cruise_norm

            v_real = v_cmd * scale_v_mps
            drive_time = seg_len / max(v_real, 1e-6)

            print(f"[FOLLOW] Segment {i}: DRIVE len={seg_len:.3f}m, v={v_cmd:.2f}, time={drive_time:.2f}s")

            # omega=0, 직진 실시간 적분
            run_for_duration(
                v_norm=v_cmd,
                omega_norm=0.0,
                v_mps=v_real,
                omega_rad=0.0,
                duration=drive_time,
            )

            time.sleep(0.05)
            
            if is_enemy_goal and i == num_segments - 1:
                TTLServo.servoAngleCtrl(2, 0, 1, 100)
                TTLServo.servoAngleCtrl(3, 90, 1, 180)
                turn_dir = 1.0
                omega_real = omega_turn_norm * scale_omega_rad      # rad/s
                turn_time = abs(math.radians(160.0)) / max(omega_real, 1e-6)
                run_for_duration(
                    v_norm=0.0,
                    omega_norm=turn_dir * omega_turn_norm,
                    v_mps=0.0,
                    omega_rad=turn_dir * omega_real,
                    duration=turn_time,
                )
                TTLServo.servoAngleCtrl(2, -100, 1, 1000)
                TTLServo.servoAngleCtrl(3, 120, 1, 1000)
                sleep(2)
                TTLServo.servoAngleCtrl(1, 0, 1, 180)
                TTLServo.servoAngleCtrl(2, -80, 1, 180)
                TTLServo.servoAngleCtrl(3, -60, 1, 180)
                TTLServo.servoAngleCtrl(4, -35, 1, 180)
                sleep(2)
                print('ready!')

    finally:
        robot.stop()

    x, y, th = pose_est.get_pose()
    print(f"[FOLLOW] Finished. est pose=({x:.2f},{y:.2f},theta_deg={math.degrees(th):.1f})")
    return (x, y, th)

def main():
    def run_for_duration2(v_norm: float, omega_norm: float, v_mps: float, omega_rad: float, duration: float):
        """
        duration 동안 step_sec 간격으로:
        - 모터 명령 유지
        - pose 적분
        - 콜백 호출
        """
        step_sec: float = 0.02
        t_end = time.monotonic() + duration
        last_t = time.monotonic()

        # 모터 명령 시작
        robot.set_velocity(v_norm, omega_norm)

        while True:
            now = time.monotonic()
            if now >= t_end:
                break

            dt = now - last_t
            last_t = now
            if dt <= 0:
                time.sleep(step_sec)
                continue

            #pose_est.update(v_mps, omega_rad, dt)
            #if pose_update_cb:
                #pose_update_cb(*pose_est.get_pose())

            # 일정 주기 유지
            sleep_t = max(0.0, step_sec - (time.monotonic() - now))
            if sleep_t > 0:
                time.sleep(sleep_t)

        # 마지막 잔여시간 보정(끝 경계까지)
        now2 = time.monotonic()
        if now2 < t_end:
            dt = t_end - now2
            #pose_est.update(v_mps, omega_rad, dt)
            #if pose_update_cb:
                #pose_update_cb(*pose_est.get_pose())

        robot.stop()
    
    robot = JetankRobot()
    omega_turn_norm : float = 0.5
    scale_omega_rad : float = 0.85
        
    TTLServo.servoAngleCtrl(2, 0, 1, 100)
    TTLServo.servoAngleCtrl(3, 90, 1, 180)
    turn_dir = 1.0
    omega_real = omega_turn_norm * scale_omega_rad      # rad/s
    turn_time = abs(math.radians(180.0)) / max(omega_real, 1e-6)
    run_for_duration2(
        v_norm=0.0,
        omega_norm=turn_dir * omega_turn_norm,
        v_mps=0.0,
        omega_rad=turn_dir * omega_real,
        duration=turn_time,
    )
    #TTLServo.servoAngleCtrl(2, -100, 1, 1000)
    #TTLServo.servoAngleCtrl(3, 120, 1, 1000)
    sleep(1)
    TTLServo.servoAngleCtrl(1, 0, 1, 180)
    TTLServo.servoAngleCtrl(2, -80, 1, 180)
    TTLServo.servoAngleCtrl(3, -60, 1, 180)
    TTLServo.servoAngleCtrl(4, -40, 1, 180)
    sleep(2)
    print('ready!')
    
if __name__ == "__main__":
    main()
