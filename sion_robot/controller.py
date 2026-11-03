import logging
import math
import os
import sys
import threading
import time
from typing import Optional

from .config import (
    CAMERA_POLL_SEC, INFLATE_RADIUS_CELLS, MAP_FILE, MIN_CHARGE_CELLS, MOTION_STEP_SEC, OMEGA_TURN_NORM,
    POSE_PERIOD_SEC, SCALE_OMEGA_RAD, SCALE_V_MPS, V_CHARGE_NORM, V_CRUISE_NORM,
)
from .follow_path import follow_path_constant_speed
from .hardware import RobotBase, create_robot
from .map_loader import MapData, cell_to_world, load_map, world_to_cell
from .mode_manager import ModeManager
from .path_planner import plan_path_for_goal
from .state import MODE_MANUAL, AGVState, Cell

logger = logging.getLogger(__name__)

# OpenCV가 없는 데스크톱(DummyRobot)에서도 제어 루프는 돌도록 카메라 스레드만 건너뛴다.
try:
    import cv2
except ImportError:  # pragma: no cover - 설치 여부에 따라 갈림
    cv2 = None

MAIN_LOOP_SEC = 0.01
CAMERA_RETRY_SEC = 1.0
CAMERA_READ_FAIL_SEC = 0.2
MOVE_JOIN_TIMEOUT_SEC = 2.0


def camera_thread_main(stop_event: threading.Event) -> None:
    """카메라 keep-alive. 어떤 예외도 삼키고 재시도해 메인 루프를 죽이지 않는다."""
    if cv2 is None:
        logger.warning("OpenCV(cv2) not installed — camera thread disabled")
        return
    logger.info("thread start")
    cap = None
    while not stop_event.is_set():
        try:
            if cap is None:
                cap = cv2.VideoCapture(0)
                if not cap.isOpened():
                    logger.warning("open failed, retry in %.0fs", CAMERA_RETRY_SEC)
                    cap = None
                    time.sleep(CAMERA_RETRY_SEC)
                    continue
                logger.info("camera opened")
            ok, _ = cap.read()
            if not ok:
                time.sleep(CAMERA_READ_FAIL_SEC)
                cap = _safe_release(cap)
                continue
            time.sleep(CAMERA_POLL_SEC)
        except (cv2.error, OSError) as e:
            logger.warning("camera I/O error: %s", e)
            cap = _safe_release(cap)
            time.sleep(CAMERA_RETRY_SEC)
        except Exception:
            logger.exception("camera unexpected exception")
            cap = _safe_release(cap)
            time.sleep(CAMERA_RETRY_SEC)
    _safe_release(cap)
    logger.info("camera thread stop")


def _safe_release(cap) -> None:
    if cap is None:
        return None
    try:
        cap.release()
    except Exception as release_err:
        logger.debug("cap.release() failed: %s", release_err)
    return None


def move_worker(
    robot: RobotBase,
    m: MapData,
    start_cell: Cell,
    goal_cell: Cell,
    is_enemy_goal: bool,
    state: AGVState,
    cancel_event: Optional[threading.Event] = None,
) -> Cell:
    """이동 후 로봇이 있다고 볼 셀을 반환한다.

    계획에 실패하면 움직이지 않았으니 start_cell, 추종을 시작했으면 개루프라 도착했다고 보고 goal_cell.
    """
    state.set_plan_result(False)

    try:
        path_cells, _ = plan_path_for_goal(
            m,
            start_cell=start_cell,
            goal_cell=goal_cell,
            is_enemy_goal=is_enemy_goal,
            inflate_radius=INFLATE_RADIUS_CELLS,
            min_charge_cells=MIN_CHARGE_CELLS,
        )
    except Exception as e:
        state.set_plan_result(False, f"plan exception: {e}")
        logger.exception("move_worker: plan_path_for_goal failed")
        return start_cell

    if path_cells is None:
        state.set_plan_result(False, "No path found")
        logger.warning("No path found")
        return start_cell

    for cx, cy in path_cells:
        logger.debug("  cell (%2d,%2d)", cx, cy)
    waypoints_world = [cell_to_world(x, y, m.resolution) for x, y in path_cells]
    state.set_plan_result(True)

    def pose_cb(px: float, py: float, ptheta: float) -> None:
        state.set_pose((px, py, ptheta), world_to_cell(px, py, m.resolution))

    try:
        follow_path_constant_speed(
            robot=robot,
            waypoints_world=waypoints_world,
            is_enemy_goal=is_enemy_goal,
            v_cruise_norm=V_CRUISE_NORM,
            v_charge_norm=V_CHARGE_NORM,
            omega_turn_norm=OMEGA_TURN_NORM,
            scale_v_mps=SCALE_V_MPS,
            scale_omega_rad=SCALE_OMEGA_RAD,
            pose_update_cb=pose_cb,
            step_sec=MOTION_STEP_SEC,
            cancel_event=cancel_event,
            theta0=_last_heading(state),
        )
    except Exception as e:
        state.set_plan_result(False, f"follow exception: {e}")
        logger.exception("move_worker: follow_path_constant_speed failed")
    return goal_cell


def _last_heading(state: AGVState) -> float:
    pose_world = state.get_pose_world()
    return pose_world[2] if pose_world is not None else 0.0


class MoveJob:

    def __init__(self, robot: RobotBase, m: MapData, start_cell: Cell, goal_cell: Cell,
                 is_enemy_goal: bool, state: AGVState, cancel_event: threading.Event):
        self.start_cell = start_cell
        self.goal_cell = goal_cell
        self.end_cell: Cell = start_cell
        self._args = (robot, m, start_cell, goal_cell, is_enemy_goal, state, cancel_event)
        self.thread = threading.Thread(target=self._run, daemon=True)

    def _run(self) -> None:
        self.end_cell = move_worker(*self._args)

    def start(self) -> None:
        self.thread.start()

    def is_alive(self) -> bool:
        return self.thread.is_alive()

    def join(self, timeout: Optional[float] = None) -> None:
        self.thread.join(timeout)


def setup_logging() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(name)s] %(levelname)s: %(message)s",
        datefmt="%H:%M:%S",
    )
    os.environ.setdefault("PYTHONIOENCODING", "utf-8")
    try:
        sys.stdout.reconfigure(encoding="utf-8")
    except Exception:
        logger.debug("stdout reconfigure not supported on this platform")


def init_state(m: MapData) -> AGVState:
    if m.start is None:
        raise RuntimeError("Start(S) not found in map")

    state = AGVState()
    if m.enemies:
        state.set_mode(MODE_MANUAL)
        state.set_pending_goal_cell(tuple(m.enemies[0]))
        state.set_enemy_goal(True)

    wx0, wy0 = cell_to_world(m.start[0], m.start[1], m.resolution)
    state.set_pose((wx0, wy0, 0.0), m.start)
    return state


def _log_pose(state: AGVState) -> None:
    pose_world = state.get_pose_world()
    pose_cell = state.get_pose_cell()
    if pose_world and pose_cell:
        px, py, pth = pose_world
        cx, cy = pose_cell
        logger.debug("world=(%.2f,%.2f,theta_deg=%.1f) cell=(%s,%s)", px, py, math.degrees(pth), cx, cy)
    else:
        logger.debug("pose is None")


def _shutdown(robot: RobotBase, move_th: Optional[MoveJob]) -> None:
    """stop_event를 set한 뒤 부른다. 이동 스레드가 취소에 반응해 모터를 멈출 시간을 주고 한 번 더 정지한다."""
    if move_th is not None and move_th.is_alive():
        move_th.join(timeout=MOVE_JOIN_TIMEOUT_SEC)
        if move_th.is_alive():
            logger.warning("move_th did not finish within %.0fs", MOVE_JOIN_TIMEOUT_SEC)
    try:
        robot.stop()
    except Exception:
        logger.exception("final robot.stop() failed")


def main() -> None:
    setup_logging()

    logger.info("Loading map ...")
    m = load_map(MAP_FILE)
    state = init_state(m)
    robot = create_robot()
    mode_mgr = ModeManager(state, m)
    stop_event = threading.Event()

    current_cell = m.start

    cam_th = threading.Thread(target=camera_thread_main, args=(stop_event,), daemon=True)
    cam_th.start()

    move_th: Optional[MoveJob] = None
    last_pose_print = 0.0

    logger.info("Enter loop (Ctrl+C to stop)")
    try:
        while True:
            now = time.time()
            is_moving = move_th is not None and move_th.is_alive()
            if move_th is not None and not is_moving:
                current_cell = move_th.end_cell

            mode_mgr.tick(current_cell, is_moving)

            if not is_moving:
                goal_cell, is_enemy = state.take_goal()
                if goal_cell is not None:
                    move_th = MoveJob(robot, m, current_cell, goal_cell, is_enemy, state, stop_event)
                    move_th.start()

            if now - last_pose_print >= POSE_PERIOD_SEC:
                _log_pose(state)
                last_pose_print = now

            time.sleep(MAIN_LOOP_SEC)

    except KeyboardInterrupt:
        logger.info("KeyboardInterrupt -> stopping")
    finally:
        stop_event.set()
        _shutdown(robot, move_th)
        logger.info("Exit")


if __name__ == "__main__":
    main()
