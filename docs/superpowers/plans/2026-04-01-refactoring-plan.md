# Sion AGV 리팩토링 구현 계획

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 매직넘버 제거, 하드웨어 추상화, 모듈 분리를 통해 코드 구조를 개선한다. 기존 동작은 유지.

**Architecture:** Bottom-Up 접근 — config 통합 → hardware 추상화 → plan_test 분리 → 개별 모듈 정리. 각 단계마다 동작 상태 유지.

**Tech Stack:** Python 3, jetbot, SCSCtrl (TTLServo), OpenCV

**Spec:** `docs/superpowers/specs/2026-04-01-refactoring-design.md`

---

### Task 1: Config 상수 통합

**Files:**
- Modify: `config.py`

- [ ] **Step 1: config.py에 누락된 상수 추가**

기존 상수(MAP_FILE, INFLATE_RADIUS, MIN_CHARGE_CELLS, V_CRUISE, V_CHARGE, OMEGA_TURN, SCALE_V_MPS, SCALE_OMEGA_RAD, POSE_PERIOD_SEC, AUTO_INTERVAL_SEC) 유지하고, 아래 상수를 추가:

```python
# --- Motion (추가) ---
K_OMEGA = 0.7
STEP_SEC = 0.02

# --- Servo ---
SERVO_SPEED = 1
SERVO_TIME = 180
SERVO_INIT_ANGLES = {1: 0, 2: -80, 3: -60, 4: -35}
SERVO_ATTACK_TURN_DEG = 160
SERVO_ATTACK_PREP = {2: (0, 100), 3: (90, 180)}   # {servo_id: (angle, time)}
SERVO_ATTACK_HIT = {2: -100, 3: 120}
SERVO_ATTACK_HIT_TIME = 1000
SERVO_ATTACK_PAUSE_SEC = 2.0

# --- Timing (추가) ---
SEGMENT_PAUSE_SEC = 0.05
```

- [ ] **Step 2: 커밋**

```bash
git add config.py
git commit -m "refactor: config.py에 모든 매직넘버 상수 통합"
```

---

### Task 2: hardware.py 생성

**Files:**
- Create: `hardware.py`

- [ ] **Step 1: hardware.py 작성**

`follow_path.py`의 `JetankRobot` 클래스를 기반으로 `hardware.py` 생성:

```python
import logging
from abc import ABC, abstractmethod

from config import (K_OMEGA, SERVO_SPEED, SERVO_TIME, SERVO_INIT_ANGLES)

logger = logging.getLogger(__name__)


class RobotBase(ABC):
    @abstractmethod
    def set_velocity(self, v_norm: float, omega_norm: float) -> None: ...

    @abstractmethod
    def stop(self) -> None: ...

    @abstractmethod
    def set_servo(self, servo_id: int, angle: int,
                  speed: int = SERVO_SPEED, time: int = SERVO_TIME) -> None: ...


class JetankRobot(RobotBase):
    def __init__(self):
        from jetbot import Robot
        from SCSCtrl import TTLServo
        self._robot = Robot()
        self._TTLServo = TTLServo
        for sid, angle in SERVO_INIT_ANGLES.items():
            TTLServo.servoAngleCtrl(sid, angle, SERVO_SPEED, SERVO_TIME)
        logger.info("JetankRobot ready")

    def set_velocity(self, v_norm: float, omega_norm: float) -> None:
        v_norm = max(-1.0, min(1.0, v_norm))
        omega_norm = max(-1.0, min(1.0, omega_norm))
        left = v_norm - K_OMEGA * omega_norm
        right = v_norm + K_OMEGA * omega_norm
        left = float(max(-1.0, min(1.0, left)))
        right = float(max(-1.0, min(1.0, right)))
        self._robot.set_motors(left, right)

    def stop(self) -> None:
        self._robot.set_motors(0.0, 0.0)

    def set_servo(self, servo_id: int, angle: int,
                  speed: int = SERVO_SPEED, time: int = SERVO_TIME) -> None:
        self._TTLServo.servoAngleCtrl(servo_id, angle, speed, time)


class DummyRobot(RobotBase):
    def __init__(self):
        logger.warning("DummyRobot: no hardware, logging only")

    def set_velocity(self, v_norm: float, omega_norm: float) -> None:
        logger.debug("(dummy) set_velocity v=%.2f omega=%.2f", v_norm, omega_norm)

    def stop(self) -> None:
        logger.debug("(dummy) stop")

    def set_servo(self, servo_id: int, angle: int,
                  speed: int = SERVO_SPEED, time: int = SERVO_TIME) -> None:
        logger.debug("(dummy) servo %d -> %d (speed=%d, time=%d)",
                     servo_id, angle, speed, time)


def create_robot(dummy: bool = False) -> RobotBase:
    if dummy:
        return DummyRobot()
    try:
        return JetankRobot()
    except ImportError:
        logger.warning("Hardware not available, falling back to DummyRobot")
        return DummyRobot()
```

- [ ] **Step 2: 커밋**

```bash
git add hardware.py
git commit -m "refactor: hardware.py 추가 — RobotBase/JetankRobot/DummyRobot 추상화"
```

---

### Task 3: follow_path.py 리팩토링

**Files:**
- Modify: `follow_path.py`

- [ ] **Step 1: JetankRobot 클래스 삭제 + import 정리**

삭제할 코드:
- `from time import sleep` (line 4)
- `from SCSCtrl import TTLServo` (line 6)
- `try: from jetbot import Robot ... except: Robot = None` (lines 10-14)
- `class JetankRobot` 전체 (lines 21-64)

추가할 import:
```python
from hardware import RobotBase
from config import (
    STEP_SEC, SEGMENT_PAUSE_SEC,
    SERVO_SPEED, SERVO_TIME, SERVO_INIT_ANGLES,
    SERVO_ATTACK_TURN_DEG, SERVO_ATTACK_PREP,
    SERVO_ATTACK_HIT, SERVO_ATTACK_HIT_TIME, SERVO_ATTACK_PAUSE_SEC,
)
```

- [ ] **Step 2: `_execute_enemy_action()` — robot 파라미터 추가 + config 상수 사용**

변경 전: `TTLServo.servoAngleCtrl(...)` 직접 호출, 매직넘버 사용
변경 후: `robot.set_servo(...)` 호출, config 상수 참조

```python
def _execute_enemy_action(robot: RobotBase, omega_turn_norm: float,
                          scale_omega_rad: float, run_for_duration) -> None:
    for sid, (angle, t) in SERVO_ATTACK_PREP.items():
        robot.set_servo(sid, angle, SERVO_SPEED, t)

    turn_dir = 1.0
    omega_real = omega_turn_norm * scale_omega_rad
    turn_time = abs(math.radians(SERVO_ATTACK_TURN_DEG)) / max(omega_real, 1e-6)

    run_for_duration(
        v_norm=0.0,
        omega_norm=turn_dir * omega_turn_norm,
        v_mps=0.0,
        omega_rad=turn_dir * omega_real,
        duration=turn_time,
    )

    for sid, angle in SERVO_ATTACK_HIT.items():
        robot.set_servo(sid, angle, SERVO_SPEED, SERVO_ATTACK_HIT_TIME)
    time.sleep(SERVO_ATTACK_PAUSE_SEC)

    for sid, angle in SERVO_INIT_ANGLES.items():
        robot.set_servo(sid, angle, SERVO_SPEED, SERVO_TIME)
    time.sleep(SERVO_ATTACK_PAUSE_SEC)

    logger.info("enemy action complete")
```

- [ ] **Step 3: `follow_path_constant_speed()` — robot 파라미터 주입 + 매직넘버 제거**

시그니처 변경:
```python
def follow_path_constant_speed(
    robot: RobotBase,               # 추가
    waypoints_world: ...,
    ...
    step_sec: float = STEP_SEC,     # 기본값을 config에서
) -> ...:
```

내부 변경:
- `robot = JetankRobot()` 라인 삭제 (파라미터로 받음)
- `time.sleep(0.05)` → `time.sleep(SEGMENT_PAUSE_SEC)`
- `_execute_enemy_action(omega_turn_norm, ...)` → `_execute_enemy_action(robot, omega_turn_norm, ...)`

- [ ] **Step 4: 커밋**

```bash
git add follow_path.py
git commit -m "refactor: follow_path.py — 하드웨어 직접 의존 제거, config 상수 사용"
```

---

### Task 4: plan_test.py → path_planner.py + show_path.py 분리

**Files:**
- Create: `path_planner.py`
- Create: `show_path.py`
- Delete: `plan_test.py`

- [ ] **Step 1: path_planner.py 생성**

`plan_test.py`에서 `plan_path_for_goal()` 함수만 추출:

```python
# path_planner.py — 경로 계획 파이프라인

from typing import List, Tuple, Optional

from map_loader import MapData
from astar import astar_normal, astar_enemy
from path_utils import smooth_path_cells, inflate_map, attach_charge_segment


def plan_path_for_goal(
    m_orig: MapData,
    start_cell: Tuple[int, int],
    goal_cell: Tuple[int, int],
    is_enemy_goal: bool,
    inflate_radius: int = 2,
    min_charge_cells: int = 8,
) -> Tuple[Optional[List[Tuple[int, int]]], bool]:
    m = inflate_map(m_orig, radius_cells=inflate_radius)
    use_enemy_mode = is_enemy_goal

    if use_enemy_mode:
        raw_path = astar_enemy(m, start_cell, goal_cell)
    else:
        raw_path = astar_normal(m, start_cell, goal_cell)

    if raw_path is None:
        return None, use_enemy_mode

    smooth_path = smooth_path_cells(m, raw_path)

    if use_enemy_mode:
        final_path = attach_charge_segment(
            m_orig, smooth_path, min_charge_cells=min_charge_cells
        )
    else:
        final_path = smooth_path

    return final_path, use_enemy_mode
```

- [ ] **Step 2: show_path.py 생성**

`plan_test.py`에서 `print_map_with_path()`와 `main()`을 추출. `print()` → `logging` 전환:

```python
# show_path.py — 경로 시각화 유틸리티

import json
import logging
import sys
from typing import List, Tuple

from map_loader import load_map, cell_to_world
from path_planner import plan_path_for_goal
from path_utils import bresenham_line
from config import INFLATE_RADIUS, MIN_CHARGE_CELLS

logger = logging.getLogger(__name__)


def print_map_with_path(
    cells: List[str],
    path: List[Tuple[int, int]],
    title: str,
) -> None:
    logger.info("\n%s", title)
    char_grid = [list(row) for row in cells]

    if path:
        for i in range(len(path) - 1):
            (x0, y0) = path[i]
            (x1, y1) = path[i + 1]
            seg = bresenham_line((x0, y0), (x1, y1))
            for (sx, sy) in seg:
                if char_grid[sy][sx] in (".",):
                    char_grid[sy][sx] = "*"

        sx, sy = path[0]
        gx, gy = path[-1]
        char_grid[sy][sx] = "S"
        char_grid[gy][gx] = "E"

    for row in reversed(char_grid):
        logger.info("".join(row))


def main():
    logging.basicConfig(
        level=logging.INFO,
        format="%(message)s",
    )

    m_orig = load_map("map_smallroom_60x60.json")
    if m_orig.start is None:
        raise RuntimeError("Start(S) not found in map")

    start = m_orig.start

    if not m_orig.enemies:
        raise RuntimeError("Enemy(E) not found in map")
    goal_cell = m_orig.enemies[0]
    is_enemy_goal = False

    logger.info("Start cell: %s", start)
    logger.info("Goal  cell: %s", goal_cell)
    logger.info("is_enemy_goal: %s", is_enemy_goal)

    path, used_enemy_mode = plan_path_for_goal(
        m_orig,
        start_cell=start,
        goal_cell=goal_cell,
        is_enemy_goal=is_enemy_goal,
        inflate_radius=INFLATE_RADIUS,
        min_charge_cells=MIN_CHARGE_CELLS,
    )

    if path is None:
        logger.info("No path found")
        return

    logger.info("Used enemy mode: %s", used_enemy_mode)
    logger.info("Final path length (cells): %d", len(path))

    with open("map_smallroom_60x60.json", "r") as f:
        raw = json.load(f)
        cells = raw["cells"]

    print_map_with_path(cells, path, "Final path (with charge if enemy)")

    logger.info("\nFinal path (world coordinates, meters):")
    for (cx, cy) in path:
        wx, wy = cell_to_world(cx, cy, m_orig.resolution)
        logger.info("  cell (%2d,%2d) -> world (%5.2f, %5.2f)", cx, cy, wx, wy)


if __name__ == "__main__":
    main()
```

- [ ] **Step 3: 커밋** (plan_test.py는 아직 삭제하지 않음 — Task 5, 6에서 import 변경 후 삭제)

```bash
git add path_planner.py show_path.py
git commit -m "refactor: path_planner.py + show_path.py 생성 (plan_test.py에서 분리)"
```

---

### Task 5: controller.py 업데이트

**Files:**
- Modify: `controller.py`

- [ ] **Step 1: import 수정**

변경할 import:
- `from map_loader import Map, ...` → `from map_loader import MapData, ...`
- `from plan_test import plan_path_for_goal` → `from path_planner import plan_path_for_goal`
- `from config import ...`에 `STEP_SEC` 추가
- `from hardware import create_robot, RobotBase` 추가

- [ ] **Step 2: robot 생성 + follow_path 호출 변경**

`main()` 함수에서 robot 생성:
```python
robot = create_robot()
```

`move_worker()` 시그니처에 `robot: RobotBase` 추가:
```python
def move_worker(
    robot: RobotBase,
    m: MapData,
    ...
```

`follow_path_constant_speed()` 호출에 robot 전달 + step_sec config 참조:
```python
follow_path_constant_speed(
    robot=robot,
    waypoints_world=waypoints_world,
    ...
    step_sec=STEP_SEC,
)
```

`move_th` 생성 시 robot 인자 추가:
```python
move_th = threading.Thread(
    target=move_worker,
    args=(robot, m, current_cell, consumed, is_enemy, state),
    daemon=True,
)
```

- [ ] **Step 3: 타입 어노테이션 `Map` → `MapData`**

`move_worker` 파라미터 타입: `m: Map` → `m: MapData`

- [ ] **Step 4: 커밋**

```bash
git add controller.py
git commit -m "refactor: controller.py — hardware 주입, import 경로 변경, Map→MapData 수정"
```

---

### Task 6: mode_manager.py import 변경

**Files:**
- Modify: `mode_manager.py`

- [ ] **Step 1: import 변경**

```python
from plan_test import plan_path_for_goal
```
→
```python
from path_planner import plan_path_for_goal
```

- [ ] **Step 2: 커밋**

```bash
git add mode_manager.py
git commit -m "refactor: mode_manager.py — import 경로 plan_test→path_planner 변경"
```

---

### Task 7: plan_test.py 삭제

**Files:**
- Delete: `plan_test.py`

- [ ] **Step 1: plan_test.py 삭제 + 커밋**

모든 소비자(controller.py, mode_manager.py)가 path_planner로 전환 완료된 후 삭제.

```bash
git rm plan_test.py
git commit -m "refactor: plan_test.py 삭제 (path_planner.py + show_path.py로 대체 완료)"
```

---

### Task 8: 최종 검증

- [ ] **Step 1: import 체인 검증**

```bash
python -c "from controller import main; print('controller OK')"
python -c "from mode_manager import ModeManager; print('mode_manager OK')"
python -c "from follow_path import follow_path_constant_speed; print('follow_path OK')"
python -c "from path_planner import plan_path_for_goal; print('path_planner OK')"
python -c "from hardware import create_robot; print('hardware OK')"
python -c "from show_path import main; print('show_path OK')"
```

- [ ] **Step 2: plan_test.py 잔여 참조 확인**

```bash
grep -r "plan_test" --include="*.py" .
```

Expected: 결과 없음 (모든 참조 제거됨)

- [ ] **Step 3: 매직넘버 잔여 확인**

주요 매직넘버가 config 외 파일에 남아있지 않은지 확인:
```bash
grep -rn "k_omega\|0\.7\b" --include="*.py" . | grep -v config.py | grep -v ".git"
grep -rn "servoAngleCtrl" --include="*.py" . | grep -v hardware.py | grep -v ".git"
```
