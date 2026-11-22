# 기여

```bash
pip install -r requirements-dev.txt   # pytest, pytest-mock, pytest-cov
python -m pytest                      # 테스트
```

저장소 역할과 코드 지도는 `ARCHITECTURE.md`에 있습니다.

제어 코드는 `sion_robot` 패키지입니다. `scripts/`와 `python -m` 명령은 패키지를 import하므로 저장소 루트에서 실행합니다. 루트의 `controller.py`는 다른 위치에서 경로로 실행해도 됩니다.

| 명령 | 용도 |
|---|---|
| `python -m sion_robot.controller` | 제어 루프 실행(`python controller.py`와 같은 동작) |
| `python -m scripts.show_map [맵 파일]` | 맵 시각화 |
| `python -m scripts.show_path` | `MAP_FILE`의 시작점→첫 적 경로 시각화 |
| `python -m scripts.generate_smallroom_map` | `map_smallroom_60x60.json` 재생성 |
| `python -m scripts.calib_motion {drive\|turn} 속도 ...` | 모터 캘리브레이션(실기). 예: `drive 0.2 0.4 0.6` |
| `python -m scripts.motor_smoke_test` | 1초 전진 후 정지(실기) |

## A* 케이스

`tests/fixtures/pathfinding-cases.json`은 sion-api `contract/pathfinding-cases.json`의 복사본이고, `astar_normal`과 sion-api A*가 같이 통과해야 하는 케이스입니다. sion-api 쪽이 바뀌면 복사해 옵니다.

```bash
cp ../sion-api/contract/pathfinding-cases.json tests/fixtures/
# 또는
curl -fsSL https://raw.githubusercontent.com/tolelom/sion-api/main/contract/pathfinding-cases.json -o tests/fixtures/pathfinding-cases.json
```

sion-api는 양옆 칸이 모두 비어야 대각으로 움직이고, 이 저장소는 막힌 모서리도 대각으로 지납니다(팽창한 맵에서 계획합니다). 결과가 갈리는 케이스는 `corner_cutting_expected`에 이 저장소의 기대값이 있습니다.

## CI

GitHub Actions(`.github/workflows/tests.yml`)가 `main`·`master` push와 PR마다 Python 3.12로 pytest를 돌리고, `sion_robot` 커버리지가 80% 미만이면 실패합니다.
