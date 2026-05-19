# Sion (Robot)

LoL 사이온 궁극기를 구현한 AGV의 로봇 제어 코드 (Python).

- [sion-backend](https://github.com/tolelom/sion-backend) — 백엔드 서버
- [sion-frontend](https://github.com/tolelom/sion-frontend) — 웹 대시보드

## Tech Stack

- Python 3.12
- A* 경로 탐색 (8방향, 적 추적용 회전 최소화 변형 포함)
- 멀티스레드 (메인 / 모션 / WebSocket)
- `pytest` + `pytest-mock` + `pytest-cov` (커버리지 91%)

## 실행

```bash
pip install -r requirements-dev.txt
python controller.py
```

> 이전의 `main_controller.py`는 삭제됐습니다. `controller.py`가 새 엔트리이며, 책임을 `state.py` / `mode_manager.py`로 분리한 슬림 오케스트레이터입니다.

### 하드웨어 없이 실행

`hardware.py`의 `create_robot(dummy=True)` 또는 jetbot/SCSCtrl 패키지가 import 실패하면 자동으로 `DummyRobot`으로 폴백됩니다. 로그만 찍고 모터는 동작하지 않으니 데스크탑에서 안전하게 실행 가능합니다.

## Tests

```bash
pip install -r requirements-dev.txt
python -m pytest tests/                          # 101 tests
python -m pytest tests/ --cov=astar --cov=path_utils --cov=map_loader \
                       --cov=state --cov=mode_manager --cov=follow_path \
                       --cov=path_planner        # 커버리지 91%
python -m pytest -m unit                         # 특정 마커만
```

CI는 매 push마다 pytest를 돌리고 커버리지가 80% 미만이면 실패시킵니다.

### 테스트 구조

| 파일 | 대상 모듈 | 케이스 | 모킹 |
|---|---|---|---|
| `tests/test_astar.py` | `astar.py` | 13 | 없음 (순수 함수) |
| `tests/test_path_utils.py` | `path_utils.py` | 21 | 없음 |
| `tests/test_map_loader.py` | `map_loader.py` | 10 | 없음 |
| `tests/test_state.py` | `state.py` | 18 | 멀티스레드 검증 |
| `tests/test_mode_manager.py` | `mode_manager.py` | 22 | `plan_path_for_goal`, `random` |
| `tests/test_follow_path.py` | `follow_path.py` | 13 | `SpyRobot` (RobotBase) |
| `tests/test_path_planner.py` | `path_planner.py` | 4 | 없음 |

`tests/conftest.py`의 `make_map(grid_str)` 헬퍼로 문자열 그리드에서 `MapData`를 만들 수 있습니다 (`'.'`=빈칸, `'#'`=벽, `'E'`=적, `'S'`=시작).

## 모듈 구조

| 파일 | 책임 |
|---|---|
| `controller.py` | 스레드 오케스트레이터 (메인 진입점) |
| `state.py` | `AGVState` — 스레드 안전 공유 상태 (lock 기반) |
| `mode_manager.py` | auto/manual 모드 전환, 랜덤 목표 선정 |
| `hardware.py` | `RobotBase` 인터페이스 + `JetankRobot` / `DummyRobot` |
| `follow_path.py` | `PoseEstimator` + 경로 따라가기 (협조적 cancel 지원) |
| `path_planner.py` | inflate → A* → smooth → (적이면 charge segment) 파이프라인 |
| `astar.py` | `astar_normal` + `astar_enemy` (회전 최소화) + `plan_path` dispatch |
| `path_utils.py` | LoS, smoothing, inflation, Bresenham, charge segment |
| `map_loader.py` | JSON 맵 → `MapData` + 셀↔월드 좌표 변환 |
| `config.py` | 모든 상수 (V_CRUISE, V_CHARGE, INFLATE_RADIUS, 서보 각도 등) |
| `tolelom/web_socket_client.py` | async WS 클라이언트 (재연결) |
| `tolelom/main.py` | WS 클라이언트 단독 실행 |
| `show_map.py` / `show_path.py` / `generate_smallroom_map.py` | 시각화/디버깅 도우미 |
| `calib_motion.py` | 모터 캘리브레이션 스크립트 |

## 주요 상수 (`config.py`)

```python
V_CRUISE = 0.35           # 일반 주행 속도 (m/s)
V_CHARGE = 0.8            # 적 돌진 속도 (m/s)
INFLATE_RADIUS = 2        # 장애물 인플레이션 (셀)
MIN_CHARGE_CELLS = 8      # 돌진 구간 최소 길이
POSE_PERIOD_SEC = 0.5     # pose 갱신 주기
AUTO_INTERVAL_SEC = 5.0   # 자동 모드 목표 재선정 간격
```
