# mode_manager.py — auto/manual 모드 전환 로직
# main_controller.py에 분산되어 있던 모드 전환 조건들을 한 곳에 집중

import random
import time
from typing import Optional, Tuple

from config import AUTO_INTERVAL_SEC, INFLATE_RADIUS, MIN_CHARGE_CELLS
from plan_test import plan_path_for_goal
from state import AGVState


class ModeManager:

    def __init__(self, state: AGVState, m):
        self._state = state
        self._map = m
        self._last_auto_time = time.time()

    def apply_server_command(self, cmd: dict, is_moving: bool) -> None:
        if "mode" in cmd and cmd["mode"] in ("auto", "manual"):
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

    def tick(self, current_cell: Tuple[int, int], is_moving: bool) -> None:
        mode = self._state.get_mode()

        if mode == "manual":
            if not is_moving and self._state.get_goal_cell() is None:
                self._state.promote_pending_goal()

        elif mode == "auto":
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

    def pick_random_goal(self, start_cell: Tuple[int, int]) -> Optional[Tuple[int, int]]:
        m = self._map
        for _ in range(80):
            gx = random.randint(0, m.width - 1)
            gy = random.randint(0, m.height - 1)
            goal = (gx, gy)
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

    def _parse_goal(self, cmd: dict):
        from map_loader import world_to_cell
        try:
            is_enemy = bool(cmd.get("is_enemy_goal", False))
            goal = cmd.get("goal")
            if goal is None:
                return None, is_enemy, "no goal in command"
            gtype = goal.get("type", "cell")
            if gtype == "cell":
                gx, gy = int(goal["x"]), int(goal["y"])
                m = self._map
                if not (0 <= gx < m.width and 0 <= gy < m.height):
                    return None, is_enemy, "goal cell out of map range"
                return (gx, gy), is_enemy, ""
            elif gtype == "world":
                wx, wy = float(goal["x"]), float(goal["y"])
                cx, cy = world_to_cell(wx, wy, self._map.resolution)
                m = self._map
                if not (0 <= cx < m.width and 0 <= cy < m.height):
                    return None, is_enemy, "goal world->cell out of map range"
                return (cx, cy), is_enemy, ""
            else:
                return None, is_enemy, f"unknown goal.type={gtype}"
        except Exception as e:
            return None, False, f"parse error: {e}"
