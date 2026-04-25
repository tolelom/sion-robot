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
from typing import Optional, Tuple

logger = logging.getLogger(__name__)

import cv2

from follow_path import follow_path_constant_speed
from map_loader import MapData, cell_to_world, load_map, world_to_cell
from config import (MAP_FILE, POSE_PERIOD_SEC, INFLATE_RADIUS, MIN_CHARGE_CELLS,
                     V_CRUISE, V_CHARGE, OMEGA_TURN, SCALE_V_MPS, SCALE_OMEGA_RAD,
                     STEP_SEC, CAMERA_POLL_SEC)
from hardware import create_robot, RobotBase
from mode_manager import ModeManager
from path_planner import plan_path_for_goal
from state import AGVState


# =========================
# CAMERA THREAD
# =========================

def camera_thread_main(stop_event: threading.Event) -> None:
    """카메라 keep-alive 스레드. 실패해도 메인을 죽이지 않음.

    데몬 스레드라 어떤 예외든 무는 것이 옳지만, 종류별로 메시지 강도를 다르게 한다.
    예상되는 실패(cv2.error / OSError)는 warning, 그 외는 traceback과 함께 error.
    """
    logger.info("thread start")
    cap = None
    while not stop_event.is_set():
        try:
            if cap is None:
                cap = cv2.VideoCapture(0)
                if not cap.isOpened():
                    logger.warning("open failed, retry in 1s")
                    cap = None
                    time.sleep(1.0)
                    continue
                logger.info("camera opened")
            ok, _ = cap.read()
            if not ok:
                time.sleep(0.2)
                cap.release()
                cap = None
                continue
            time.sleep(CAMERA_POLL_SEC)
        except (cv2.error, OSError) as e:
            logger.warning("camera I/O error: %s", e)
            cap = _safe_release(cap)
            time.sleep(1.0)
        except Exception:
            logger.exception("camera unexpected exception")
            cap = _safe_release(cap)
            time.sleep(1.0)
    _safe_release(cap)
    logger.info("camera thread stop")


def _safe_release(cap) -> None:
    """cv2 캡처 자원 해제. 실패해도 호출부를 깨지 않는다."""
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
    start_cell: Tuple[int, int],
    goal_cell: Tuple[int, int],
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
        cx, cy = world_to_cell(px, py, m.resolution)
        state.set_pose((px, py, ptheta), (cx, cy))

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

def main() -> None:
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

    logger.info("Loading map ...")
    m = load_map(MAP_FILE)
    if m.start is None:
        raise RuntimeError("Start(S) not found in map")

    state = AGVState()
    robot = create_robot()
    mode_mgr = ModeManager(state, m)
    stop_event = threading.Event()

    # 적 좌표가 있으면 초기 목표로 설정
    if hasattr(m, "enemies") and m.enemies:
        enemy_cell = tuple(m.enemies[0])
        state.set_mode("manual")
        state.set_pending_goal_cell(enemy_cell)
        state.set_enemy_goal(True)

    current_cell = m.start
    wx0, wy0 = cell_to_world(current_cell[0], current_cell[1], m.resolution)
    state.set_pose((wx0, wy0, 0.0), current_cell)

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

            # 목표가 있으면 이동 시작
            goal_cell = state.get_goal_cell()
            if not is_moving and goal_cell is not None:
                is_enemy = state.is_enemy_goal()
                consumed = state.consume_goal()
                if consumed is not None:
                    move_th = threading.Thread(
                        target=move_worker,
                        args=(robot, m, current_cell, consumed, is_enemy, state, stop_event),
                        daemon=True,
                    )
                    move_th.start()
                    current_cell = consumed

            # 주기적 포즈 출력
            if now - last_pose_print >= POSE_PERIOD_SEC:
                pose_world = state.get_pose_world()
                pose_cell = state.get_pose_cell()
                if pose_world and pose_cell:
                    px, py, pth = pose_world
                    cx, cy = pose_cell
                    logger.debug("world=(%.2f,%.2f,theta_deg=%.1f) cell=(%s,%s)", px, py, math.degrees(pth), cx, cy)
                else:
                    logger.debug("pose is None")
                last_pose_print = now

            time.sleep(0.01)

    except KeyboardInterrupt:
        logger.info("KeyboardInterrupt -> stopping")
    finally:
        stop_event.set()
        # 진행 중인 follow가 cancel_event(stop_event)에 반응해 모터 정지 + 반환할 시간을 준다.
        # daemon thread라 join 없이도 프로세스 종료에 묶이지만, robot.stop() 보장을 위해 join.
        if move_th is not None and move_th.is_alive():
            move_th.join(timeout=2.0)
            if move_th.is_alive():
                logger.warning("move_th did not finish within 2s")
        # 안전 차원에서 한 번 더 모터 정지 시도.
        try:
            robot.stop()
        except Exception:
            logger.exception("final robot.stop() failed")
        logger.info("Exit")


if __name__ == "__main__":
    main()
