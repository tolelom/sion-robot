# config.py — 공유 상수 중앙 관리
import os

# 저장소 루트. 데이터 파일 경로는 실행 위치(cwd)가 아니라 이 위치 기준으로 찾는다.
BASE_DIR = os.path.dirname(os.path.abspath(__file__))

# --- Map ---
MAP_FILE = os.path.join(BASE_DIR, "map_smallroom_60x60.json")

# --- Planning ---
INFLATE_RADIUS = 2
MIN_CHARGE_CELLS = 8

# --- Motion ---
V_CRUISE = 0.35
V_CHARGE = 0.8
OMEGA_TURN = 0.5
SCALE_V_MPS = 0.15
SCALE_OMEGA_RAD = 0.85
K_OMEGA = 0.7
STEP_SEC = 0.02

# --- Servo ---
SERVO_SPEED = 1
SERVO_TIME = 180
SERVO_INIT_ANGLES = {1: 0, 2: -80, 3: -60, 4: -35}
SERVO_ATTACK_TURN_DEG = 160
SERVO_ATTACK_PREP = {2: (0, 100), 3: (90, 180)}
SERVO_ATTACK_HIT = {2: -100, 3: 120}
SERVO_ATTACK_HIT_TIME = 1000
SERVO_ATTACK_PAUSE_SEC = 2.0

# --- Timing ---
POSE_PERIOD_SEC = 0.5
AUTO_INTERVAL_SEC = 5.0
SEGMENT_PAUSE_SEC = 0.05
CAMERA_POLL_SEC = 0.02
