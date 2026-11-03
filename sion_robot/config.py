import os

# 데이터 파일은 실행 위치(cwd)가 아니라 저장소 루트 기준으로 찾는다.
BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

MAP_FILE = os.path.join(BASE_DIR, "map_smallroom_60x60.json")

INFLATE_RADIUS_CELLS = 2
MIN_CHARGE_CELLS = 8

V_CRUISE_NORM = 0.35
V_CHARGE_NORM = 0.8
OMEGA_TURN_NORM = 0.5
# 정규화 속도 1.0일 때 실제 선속도(m/s)·각속도(rad/s). scripts.calib_motion으로 잰다.
SCALE_V_MPS = 0.15
SCALE_OMEGA_RAD = 0.85
OMEGA_MIX_GAIN = 0.7
MOTION_STEP_SEC = 0.02

# TTLServo.servoAngleCtrl의 3·4번째 인자로 그대로 넘기는 서보 라이브러리 고유 값.
SERVO_SPEED = 1
SERVO_TIME = 180
SERVO_INIT_ANGLES = {1: 0, 2: -80, 3: -60, 4: -35}
SERVO_ATTACK_TURN_DEG = 160
SERVO_ATTACK_PREP = {2: (0, 100), 3: (90, 180)}  # servo_id -> (angle, time)
SERVO_ATTACK_HIT_ANGLES = {2: -100, 3: 120}
SERVO_ATTACK_HIT_TIME = 1000
SERVO_ATTACK_PAUSE_SEC = 2.0

POSE_PERIOD_SEC = 0.5
AUTO_INTERVAL_SEC = 5.0
SEGMENT_PAUSE_SEC = 0.05
CAMERA_POLL_SEC = 0.02
