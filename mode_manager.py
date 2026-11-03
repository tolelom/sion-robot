# mode_manager.py — auto/manual 모드 전환 로직
# 모드 전환 조건과 목표 선정을 한 곳에 모은다. 상태 저장은 AGVState가 맡는다.

import random
import time
from typing import Any, Dict, Optional, Tuple

from config import AUTO_INTERVAL_SEC, INFLATE_RADIUS, MIN_CHARGE_CELLS
from map_loader import MapData, world_to_cell
from path_planner import plan_path_for_goal
from state import MODE_AUTO, MODE_MANUAL, VALID_MODES, AGVState, Cell

# 무작위 목표를 고를 때 경로가 있는 셀을 찾기까지 시도하는 최대 횟수
RANDOM_GOAL_MAX_TRIES = 80

# (목표 셀 또는 None, 적 목표 여부, 오류 메시지 — 정상이면 빈 문자열)
ParsedGoal = Tuple[Optional[Cell], bool, str]


class ModeManager:

    def __init__(self, state: AGVState, m: MapData):
        self._state = state
        self._map = m
        self._last_auto_time = time.time()

    def apply_server_command(self, cmd: Dict[str, Any], is_moving: bool) -> None:
        """서버 명령(dict)을 해석해 모드·목표를 갱신한다. 이동 중이면 목표는 pending으로 보류."""
        if cmd.get("mode") in VALID_MODES:
            self._state.set_mode(cmd["mode"])

        goal_cell, is_enemy, err = self._parse_goal(cmd)
        if err:
            self._state.set_plan_result(False, err)
            return

        self._state.set_cmd_timestamp(time.time())
        self._state.set_enemy_goal(is_enemy)
        if goal_cell is not None:
            if is_moving:
                self._state.set_pending_goal_cell(goal_cell)
            else:
                self._state.set_goal_cell(goal_cell)

    def tick(self, current_cell: Cell, is_moving: bool) -> None:
        """메인 루프에서 매 cycle 호출. 모드에 따라 다음 목표를 정한다."""
        mode = self._state.get_mode()

        if mode == MODE_MANUAL:
            if not is_moving and self._state.get_goal_cell() is None:
                self._state.promote_pending_goal()

        elif mode == MODE_AUTO:
            self._state.set_pending_goal_cell(None)
            now = time.time()
            if (not is_moving
                    and self._state.get_goal_cell() is None
                    and now - self._last_auto_time >= AUTO_INTERVAL_SEC):
                goal = self.pick_random_goal(current_cell)
                if goal is not None:
                    self._state.set_goal_cell(goal)
                    self._state.set_enemy_goal(False)
                self._last_auto_time = now

    def pick_random_goal(self, start_cell: Cell) -> Optional[Cell]:
        """start_cell에서 경로가 존재하는 무작위 셀을 찾는다. 못 찾으면 None."""
        m = self._map
        for _ in range(RANDOM_GOAL_MAX_TRIES):
            goal = (random.randint(0, m.width - 1), random.randint(0, m.height - 1))
            if goal == start_cell:
                continue
            path_cells, _ = plan_path_for_goal(
                m, start_cell=start_cell, goal_cell=goal,
                is_enemy_goal=False,
                inflate_radius=INFLATE_RADIUS,
                min_charge_cells=MIN_CHARGE_CELLS,
            )
            if path_cells is not None:
                return goal
        return None

    def _in_map(self, x: int, y: int) -> bool:
        return 0 <= x < self._map.width and 0 <= y < self._map.height

    def _parse_goal(self, cmd: Dict[str, Any]) -> ParsedGoal:
        """cmd["goal"] = {"type": "cell"|"world", "x": ..., "y": ...} 을 셀 좌표로 바꾼다."""
        is_enemy = bool(cmd.get("is_enemy_goal", False))
        goal = cmd.get("goal")
        if not isinstance(goal, dict):
            return None, is_enemy, "no goal in command" if goal is None else "goal must be a dict"

        gtype = goal.get("type", "cell")
        gx_raw = goal.get("x")
        gy_raw = goal.get("y")
        if gx_raw is None or gy_raw is None:
            return None, is_enemy, "goal.x / goal.y missing"

        try:
            if gtype == "cell":
                gx, gy = int(gx_raw), int(gy_raw)
                if not self._in_map(gx, gy):
                    return None, is_enemy, "goal cell out of map range"
                return (gx, gy), is_enemy, ""
            elif gtype == "world":
                cx, cy = world_to_cell(float(gx_raw), float(gy_raw), self._map.resolution)
                if not self._in_map(cx, cy):
                    return None, is_enemy, "goal world->cell out of map range"
                return (cx, cy), is_enemy, ""
            else:
                return None, is_enemy, f"unknown goal.type={gtype}"
        except (TypeError, ValueError) as e:
            return None, is_enemy, f"goal coord parse error: {e}"
