# state.py — 공유 상태 전담 클래스
# 모든 스레드는 이 클래스의 메서드를 통해서만 상태를 읽고 씀

import threading
from typing import Optional, Tuple, Any, Dict


class AGVState:
    """
    스레드 안전 공유 상태.
    직접 dict 접근 대신 get_*/set_* 메서드만 사용할 것.
    """

    def __init__(self):
        self._lock = threading.Lock()
        self._state: Dict[str, Any] = {
            "mode": "auto",
            "goal_cell": None,
            "pending_goal_cell": None,
            "is_enemy_goal": False,
            "last_cmd_ts": 0.0,
            "last_plan_ok": False,
            "last_error": "",
            "pose_world": None,
            "pose_cell": None,
        }

    def get_mode(self) -> str:
        with self._lock:
            return self._state["mode"]

    def get_goal_cell(self) -> Optional[Tuple[int, int]]:
        with self._lock:
            return self._state["goal_cell"]

    def get_pending_goal_cell(self) -> Optional[Tuple[int, int]]:
        with self._lock:
            return self._state["pending_goal_cell"]

    def is_enemy_goal(self) -> bool:
        with self._lock:
            return self._state["is_enemy_goal"]

    def get_pose_world(self) -> Optional[Tuple[float, float, float]]:
        with self._lock:
            return self._state["pose_world"]

    def get_pose_cell(self) -> Optional[Tuple[int, int]]:
        with self._lock:
            return self._state["pose_cell"]

    def get_last_error(self) -> str:
        with self._lock:
            return self._state["last_error"]

    def get_snapshot(self) -> Dict[str, Any]:
        with self._lock:
            return dict(self._state)

    def set_mode(self, mode: str) -> None:
        assert mode in ("auto", "manual"), f"Invalid mode: {mode}"
        with self._lock:
            self._state["mode"] = mode

    def set_goal_cell(self, cell: Optional[Tuple[int, int]]) -> None:
        with self._lock:
            self._state["goal_cell"] = cell

    def set_pending_goal_cell(self, cell: Optional[Tuple[int, int]]) -> None:
        with self._lock:
            self._state["pending_goal_cell"] = cell

    def set_enemy_goal(self, value: bool) -> None:
        with self._lock:
            self._state["is_enemy_goal"] = value

    def set_pose(self, world: Tuple[float, float, float], cell: Tuple[int, int]) -> None:
        with self._lock:
            self._state["pose_world"] = world
            self._state["pose_cell"] = cell

    def set_plan_result(self, ok: bool, error: str = "") -> None:
        with self._lock:
            self._state["last_plan_ok"] = ok
            self._state["last_error"] = error

    def set_cmd_timestamp(self, ts: float) -> None:
        with self._lock:
            self._state["last_cmd_ts"] = ts

    def promote_pending_goal(self) -> Optional[Tuple[int, int]]:
        with self._lock:
            pending = self._state["pending_goal_cell"]
            if pending is not None:
                self._state["goal_cell"] = pending
                self._state["pending_goal_cell"] = None
            return pending

    def consume_goal(self) -> Optional[Tuple[int, int]]:
        with self._lock:
            goal = self._state["goal_cell"]
            self._state["goal_cell"] = None
            return goal
