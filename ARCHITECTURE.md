# 아키텍처

## 시스템 구성

| 저장소 | 역할 |
|---|---|
| [sion-web](https://github.com/tolelom/sion-web) | 관제 대시보드 |
| [sion-api](https://github.com/tolelom/sion-api) | 대시보드용 WebSocket·REST 서버. 내장 시뮬레이터가 AGV 역할 |
| [sion-robot](https://github.com/tolelom/sion-robot) (이 저장소) | JETANK 제어. 경로 계획, 경로 추종, 적 돌진. 서버와 연결 없음 |

구성도는 `README.md` 맨 위에 있습니다. sion-api와 코드로 연결되지는 않고, A* 케이스 파일 하나를 sion-api에서 복사해 같이 씁니다.

## 코드 지도

| 경로 | 책임 |
|---|---|
| `sion_robot/controller.py` | 진입점. 메인 루프, 카메라 스레드, 이동 스레드 관리 |
| `sion_robot/state.py` | 스레드 간 공유 상태(lock 기반) |
| `sion_robot/mode_manager.py` | 자동/수동 모드 전환, 무작위 목표 선정 |
| `sion_robot/path_planner.py` | 팽창 → A* → 평활화 → 돌진 구간 파이프라인 |
| `sion_robot/astar.py` | 일반 A*와 적 추적용 A* |
| `sion_robot/path_utils.py` | 시야선 검사, 평활화, 팽창, Bresenham, 돌진 구간 |
| `sion_robot/follow_path.py` | 위치 추정과 경로 추종, 적 도달 동작 |
| `sion_robot/hardware.py` | 로봇 인터페이스와 JETANK·더미 구현 |
| `sion_robot/map_loader.py` | JSON 맵 로드, 셀↔월드 좌표 변환 |
| `sion_robot/config.py` | 속도, 팽창 반경, 서보 각도 등 상수 |
| `controller.py` | `python controller.py`로 실행하기 위한 진입점(`sion_robot.controller.main` 호출) |
| `map_*.json` | 격자 맵 |
| `scripts/` | 맵·경로 시각화, 맵 생성, 모터 캘리브레이션, 모터 동작 확인 |
| `tests/` | pytest 단위·통합 테스트. 하드웨어 없이 돎 |
| `tests/fixtures/pathfinding-cases.json` | sion-api `contract/pathfinding-cases.json`의 복사본 |

## 지켜야 할 규칙

- 맵 파일은 저장소 루트에 둡니다. `config.py`가 맵 경로를 저장소 루트(`BASE_DIR`) 기준 절대 경로로 만듭니다.
- `tests/`에서 맵은 `conftest.py`의 `make_map`으로 문자열 격자에서 만듭니다.
