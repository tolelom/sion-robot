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
                     V_CRUISE, V_CHARGE, OMEGA_TURN, SCALE_V_MPS, SCALE_OMEGA_RAD, STEP_SEC)
from hardware import create_robot, RobotBase
from mode_manager import ModeManager
from path_planner import plan_path_for_goal
from state import AGVState


# =========================
# CAMERA THREAD
# =========================

def camera_thread_main(stop_event: threading.Event) -> None:
    """카메라 keep-alive 스레드. 실패해도 메인을 죽이지 않음."""
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
            time.sleep(0.02)
        except Exception as e:
            logger.error("camera exception: %s", e)
            if cap is not None:
                try:
                    cap.release()
                except Exception:
                    pass
            cap = None
            time.sleep(1.0)
    if cap is not None:
        try:
            cap.release()
        except Exception:
            pass
    logger.info("camera thread stop")


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
) -> None:
    """경로 계획 → 웨이포인트 변환 → follow_path 실행"""
    try:
        state.set_plan_result(False)
        path_cells, _ = plan_path_for_goal(
            m,
            start_cell=start_cell,
            goal_cell=goal_cell,
            is_enemy_goal=is_enemy_goal,
            inflate_radius=INFLATE_RADIUS,
            min_charge_cells=MIN_CHARGE_CELLS,
        )
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
        )
    except Exception as e:
        state.set_plan_result(False, f"move_worker exception: {e}")
        logger.error("move_worker exception: %s", e)


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
        pass

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
                        args=(robot, m, current_cell, consumed, is_enemy, state),
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
        time.sleep(0.2)
        logger.info("Exit")


if __name__ == "__main__":
    main()
