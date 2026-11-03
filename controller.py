#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# controller.py — 스레드 오케스트레이션 전담
# 상태 관리는 AGVState, 모드 전환은 ModeManager에 위임

import logging
import math
import os
import sys
import threading
import time
from typing import Optional

from config import (CAMERA_POLL_SEC, INFLATE_RADIUS, MAP_FILE, MIN_CHARGE_CELLS, OMEGA_TURN,
                    POSE_PERIOD_SEC, SCALE_OMEGA_RAD, SCALE_V_MPS, STEP_SEC, V_CHARGE, V_CRUISE)
from follow_path import follow_path_constant_speed
from hardware import RobotBase, create_robot
from map_loader import MapData, cell_to_world, load_map, world_to_cell
from mode_manager import ModeManager
from path_planner import plan_path_for_goal
from state import MODE_MANUAL, AGVState, Cell

logger = logging.getLogger(__name__)

# OpenCV는 카메라 keep-alive 스레드에만 필요하다. 없으면 카메라 스레드만 건너뛰고
# 제어 루프는 그대로 돈다(데스크톱 + DummyRobot 실행용).
try:
    import cv2
except ImportError:  # pragma: no cover - 설치 여부에 따라 갈림
    cv2 = None

MAIN_LOOP_SEC = 0.01           # 메인 루프 주기
CAMERA_RETRY_SEC = 1.0         # 카메라 열기 실패·예외 후 재시도 대기
CAMERA_READ_FAIL_SEC = 0.2     # 프레임 읽기 실패 후 재오픈 전 대기
MOVE_JOIN_TIMEOUT_SEC = 2.0    # 종료 시 이동 스레드가 정지하기를 기다리는 시간


# =========================
# CAMERA THREAD
# =========================

def camera_thread_main(stop_event: threading.Event) -> None:
    """카메라 keep-alive 스레드. 실패해도 메인을 죽이지 않음.

    데몬 스레드라 어떤 예외든 무는 것이 옳지만, 종류별로 메시지 강도를 다르게 한다.
    예상되는 실패(cv2.error / OSError)는 warning, 그 외는 traceback과 함께 error.
    """
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
    """cv2 캡처 자원 해제. 실패해도 호출부를 깨지 않는다. 항상 None을 반환한다(cap = _safe_release(cap))."""
    if cap is None:
        return None
    try:
        cap.release()
    except Exception as release_err:
        logger.debug("cap.release() failed: %s", release_err)
    return None


# =========================
# MOVE WORKER
# =========================

def move_worker(
    robot: RobotBase,
    m: MapData,
    start_cell: Cell,
    goal_cell: Cell,
    is_enemy_goal: bool,
    state: AGVState,
    cancel_event: Optional[threading.Event] = None,
) -> None:
    """경로 계획 → 웨이포인트 변환 → follow_path 실행.

    plan / follow 단계를 분리해 어느 단계에서 실패했는지 로그로 식별 가능하게 한다.
    cancel_event가 set되면 follow_path가 즉시 정지하고 반환한다.
    """
    state.set_plan_result(False)

    try:
        path_cells, _ = plan_path_for_goal(
            m,
            start_cell=start_cell,
            goal_cell=goal_cell,
            is_enemy_goal=is_enemy_goal,
            inflate_radius=INFLATE_RADIUS,
            min_charge_cells=MIN_CHARGE_CELLS,
        )
    except Exception as e:
        state.set_plan_result(False, f"plan exception: {e}")
        logger.exception("move_worker: plan_path_for_goal failed")
        return

    if path_cells is None:
        state.set_plan_result(False, "No path found")
        logger.warning("No path found")
        return

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
            v_cruise_norm=V_CRUISE,
            v_charge_norm=V_CHARGE,
            omega_turn_norm=OMEGA_TURN,
            scale_v_mps=SCALE_V_MPS,
            scale_omega_rad=SCALE_OMEGA_RAD,
            pose_update_cb=pose_cb,
            step_sec=STEP_SEC,
            cancel_event=cancel_event,
        )
    except Exception as e:
        state.set_plan_result(False, f"follow exception: {e}")
        logger.exception("move_worker: follow_path_constant_speed failed")


# =========================
# MAIN
# =========================

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
    """맵의 시작 위치로 포즈를 초기화한다. 적이 있으면 첫 적을 manual 모드 목표로 예약한다."""
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


def _shutdown(robot: RobotBase, move_th: Optional[threading.Thread]) -> None:
    """stop_event가 set된 뒤 호출: 이동 스레드가 모터를 멈추고 끝나기를 기다린 뒤 한 번 더 정지."""
    # 진행 중인 follow가 cancel_event(stop_event)에 반응해 모터 정지 + 반환할 시간을 준다.
    # daemon thread라 join 없이도 프로세스 종료에 묶이지만, robot.stop() 보장을 위해 join.
    if move_th is not None and move_th.is_alive():
        move_th.join(timeout=MOVE_JOIN_TIMEOUT_SEC)
        if move_th.is_alive():
            logger.warning("move_th did not finish within %.0fs", MOVE_JOIN_TIMEOUT_SEC)
    # 안전 차원에서 한 번 더 모터 정지 시도.
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

    move_th: Optional[threading.Thread] = None
    last_pose_print = 0.0

    logger.info("Enter loop (Ctrl+C to stop)")
    try:
        while True:
            now = time.time()
            is_moving = move_th is not None and move_th.is_alive()

            # 모드 매니저 틱 — 목표 상태 갱신
            mode_mgr.tick(current_cell, is_moving)

            # 목표가 있으면 이동 시작.
            # invariant: move_th는 한 번에 하나만 살아 있다. is_moving 가드(위에서 is_alive 확인)가
            # 동시 실행을 막고, 이전 move_th 객체는 새 할당으로 해제되어 GC된다(데몬 스레드라
            # 명시 join 불필요). 새 goal은 이전 follow가 자연 종료된 다음 cycle에 picked up된다.
            if not is_moving:
                goal_cell, is_enemy = state.take_goal()
                if goal_cell is not None:
                    move_th = threading.Thread(
                        target=move_worker,
                        args=(robot, m, current_cell, goal_cell, is_enemy, state, stop_event),
                        daemon=True,
                    )
                    move_th.start()
                    current_cell = goal_cell

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
