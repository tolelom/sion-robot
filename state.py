import threading
from typing import Any, Dict, Optional, Tuple

MODE_AUTO = "auto"
MODE_MANUAL = "manual"
VALID_MODES = (MODE_AUTO, MODE_MANUAL)

Cell = Tuple[int, int]
WorldPose = Tuple[float, float, float]  # (x[m], y[m], theta[rad])


class AGVState:
    """스레드 간 공유 상태. 모든 접근은 lock을 잡는 메서드로만 한다."""

    def __init__(self):
        self._lock = threading.Lock()
        self._state: Dict[str, Any] = {
            "mode": MODE_AUTO,
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

    def get_goal_cell(self) -> Optional[Cell]:
        with self._lock:
            return self._state["goal_cell"]

    def get_pending_goal_cell(self) -> Optional[Cell]:
        with self._lock:
            return self._state["pending_goal_cell"]

    def is_enemy_goal(self) -> bool:
        with self._lock:
            return self._state["is_enemy_goal"]

    def get_pose_world(self) -> Optional[WorldPose]:
        with self._lock:
            return self._state["pose_world"]

    def get_pose_cell(self) -> Optional[Cell]:
        with self._lock:
            return self._state["pose_cell"]

    def get_last_error(self) -> str:
        with self._lock:
            return self._state["last_error"]

    def get_snapshot(self) -> Dict[str, Any]:
        with self._lock:
            return dict(self._state)

    def set_mode(self, mode: str) -> None:
        if mode not in VALID_MODES:
            raise ValueError(f"Invalid mode: {mode!r}")
        with self._lock:
            self._state["mode"] = mode

    def set_goal_cell(self, cell: Optional[Cell]) -> None:
        with self._lock:
            self._state["goal_cell"] = cell

    def set_pending_goal_cell(self, cell: Optional[Cell]) -> None:
        with self._lock:
            self._state["pending_goal_cell"] = cell

    def set_enemy_goal(self, value: bool) -> None:
        with self._lock:
            self._state["is_enemy_goal"] = value

    def set_pose(self, world: WorldPose, cell: Cell) -> None:
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

    def promote_pending_goal(self) -> Optional[Cell]:
        with self._lock:
            pending = self._state["pending_goal_cell"]
            if pending is not None:
                self._state["goal_cell"] = pending
                self._state["pending_goal_cell"] = None
            return pending

    def consume_goal(self) -> Optional[Cell]:
        with self._lock:
            goal = self._state["goal_cell"]
            self._state["goal_cell"] = None
            return goal

    def take_goal(self) -> Tuple[Optional[Cell], bool]:
        # consume_goal()과 is_enemy_goal()을 따로 부르면 그 사이 다른 스레드가 바꿔 짝이 어긋날 수 있어
        # 한 번의 lock 안에서 함께 읽는다.
        with self._lock:
            goal = self._state["goal_cell"]
            self._state["goal_cell"] = None
            return goal, self._state["is_enemy_goal"]
