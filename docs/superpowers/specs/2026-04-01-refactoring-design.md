# Sion AGV 코드 리팩토링 설계

## 목표

코드 정리 + 구조 개선. 기능 추가 없이 기존 동작을 유지하면서 매직넘버 제거, 모듈 경계 재정의, 하드웨어 추상화를 수행한다.

## 범위

- config.py 상수 통합
- 하드웨어 추상화 레이어 (hardware.py)
- plan_test.py 분리 (path_planner.py + show_path.py)
- 개별 모듈 정리 (import, logging, 매직넘버)

범위 밖: tolelom/ WebSocket 연결, 새 기능 추가, 테스트 코드 작성.

---

## 1. Config 통합

### 현재 문제

`config.py`에 일부 상수만 존재. 나머지는 각 모듈에 하드코딩:

| 상수 | 현재 위치 | 값 |
|------|-----------|-----|
| inflate_radius | controller.py, plan_test.py | 2 |
| step_sec | follow_path.py | 0.02 |
| k_omega | follow_path.py | 0.7 |
| servo 초기 각도 | follow_path.py | [0, -80, -60, -35] |
| servo 공격 시퀀스 | follow_path.py | 160도 회전 + 팔 애니메이션 |
| min_charge_cells | plan_test.py | 8 |

### 변경

`config.py`에 카테고리별 그룹화:

```python
# --- Map ---
MAP_FILE = "map_smallroom_60x60.json"
INFLATE_RADIUS = 2

# --- Motion ---
V_CRUISE = 0.35
V_CHARGE = 0.8
OMEGA_TURN = 0.5
K_OMEGA = 0.7
STEP_SEC = 0.02

# --- Servo ---
SERVO_INIT_ANGLES = {1: 0, 2: -80, 3: -60, 4: -35}
SERVO_ATTACK_TURN_ANGLE = 160
SERVO_ATTACK_SEQUENCE = [
    # (servo_id, angle, delay_sec)
    (1, 160, 0.3),
    (3, 30, 0.2),
    (4, 0, 0.2),
    (3, -60, 0.2),
    (4, -35, 0.2),
]

# --- Planning ---
MIN_CHARGE_CELLS = 8

# --- Timing ---
POSE_PERIOD_SEC = 0.5
AUTO_INTERVAL_SEC = 5.0
```

각 모듈에서 `from config import INFLATE_RADIUS, ...` 으로 참조.

---

## 2. 하드웨어 추상화 레이어

### 현재 문제

`follow_path.py`에서 `jetbot.Robot`, `SCSCtrl`을 직접 import. 하드웨어 없으면 `None` 폴백이지만 로직이 분산되어 있음.

### 변경

`hardware.py` 신규 생성:

```
hardware.py
├── RobotBase (ABC)
│   ├── move(speed, turn_speed)
│   ├── stop()
│   └── set_servo(servo_id, angle)
├── JetankRobot(RobotBase)
│   └── jetbot.Robot + SCSCtrl 래핑
├── DummyRobot(RobotBase)
│   └── logging만, 실제 동작 없음
└── create_robot(dummy=False) -> RobotBase
    └── dummy=False일 때 JetankRobot 시도, 실패 시 DummyRobot 폴백
```

**인터페이스:**

```python
class RobotBase(ABC):
    @abstractmethod
    def move(self, speed: float, turn_speed: float) -> None: ...

    @abstractmethod
    def stop(self) -> None: ...

    @abstractmethod
    def set_servo(self, servo_id: int, angle: int) -> None: ...
```

**JetankRobot:** 기존 `follow_path.py`의 `JetankRobot` 클래스 로직을 이동. `_velocity_to_motors()` 내부 메서드 포함.

**DummyRobot:** 모든 메서드가 `logger.debug()`만 호출. 하드웨어 없는 환경에서 경로 계획/로직 테스트용.

**create_robot():** 팩토리 함수. `dummy=True`면 DummyRobot 반환. `dummy=False`면 JetankRobot 생성 시도, import 실패 시 DummyRobot 폴백 + 경고 로그.

**영향:**
- `follow_path.py` — `JetankRobot` 클래스 삭제, `RobotBase` 인터페이스만 의존
- `controller.py` — `create_robot()` 호출 후 follow_path에 주입

---

## 3. plan_test.py 분리

### 현재 문제

`plan_test.py`에 경로 계획 파이프라인과 시각화/테스트 코드가 혼재.

### 변경

**path_planner.py** (계획 로직):
- `plan_path_for_goal()` 함수 이동
- astar, path_utils, map_loader import 유지
- config에서 상수 참조

**show_path.py** (시각화/디버깅):
- `print_map_with_path()` 함수 이동
- 기존 `main()` 테스트 코드 이동
- `print()` → `logging` 전환
- `python show_path.py`로 직접 실행 가능 유지

**plan_test.py** 삭제.

**영향받는 import:**
- `controller.py`: `from plan_test import plan_path_for_goal` → `from path_planner import plan_path_for_goal`
- `mode_manager.py`: 동일 변경

---

## 4. 개별 모듈 정리

### follow_path.py

- `JetankRobot` 클래스 삭제 (hardware.py로 이동)
- `robot` 파라미터를 `RobotBase` 타입으로 받도록 변경
- 매직넘버 → config 참조:
  - `0.02` → `config.STEP_SEC`
  - `0.7` → `config.K_OMEGA`
  - 서보 각도 → `config.SERVO_*`
- `from time import sleep` 중복 import 정리 (`time.sleep()` 사용하므로 제거)

### controller.py

- `hardware.create_robot()` 으로 robot 생성
- robot 인스턴스를 `follow_path_constant_speed()`에 전달
- `from plan_test import ...` → `from path_planner import ...`

### mode_manager.py

- `from plan_test import ...` → `from path_planner import ...`

### logging 통일

- `show_path.py`의 `print()` → `logging`
- 나머지 모듈은 변경 없음

---

## 파일 변경 요약

| 파일 | 변경 |
|------|------|
| config.py | 상수 추가 (INFLATE_RADIUS, K_OMEGA, STEP_SEC, SERVO_*, MIN_CHARGE_CELLS) |
| hardware.py | **신규** — RobotBase, JetankRobot, DummyRobot, create_robot() |
| path_planner.py | **신규** — plan_path_for_goal() (plan_test.py에서 이동) |
| show_path.py | **신규** — print_map_with_path() + main() (plan_test.py에서 이동) |
| plan_test.py | **삭제** |
| follow_path.py | JetankRobot 삭제, 매직넘버→config, robot 파라미터화 |
| controller.py | hardware.create_robot() 사용, import 경로 변경 |
| mode_manager.py | import 경로 변경 |

## 불변 사항

- astar.py — 변경 없음
- path_utils.py — 변경 없음
- map_loader.py — 변경 없음
- state.py — 변경 없음
- tolelom/ — 변경 없음
- JSON 맵 파일 — 변경 없음
