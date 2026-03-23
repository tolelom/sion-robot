#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
DEPRECATED: main_controller.py는 controller.py로 교체되었습니다.
이 파일은 호환성을 위해 유지되지만 새 개발은 controller.py를 사용하세요.
"""

import os
import sys
import math
import time
import threading
import random
from typing import Optional, Tuple, Dict, Any

import cv2

from map_loader import load_map, cell_to_world, world_to_cell
from plan_test import plan_path_for_goal
from follow_path import follow_path_constant_speed


# =========================
# CONFIG
# =========================

MAP_FILE = "map_smallroom_60x60.json"

SERVER_POLL_SEC = 0.1         # 서버 폴링 주기(지금은 stub)
POSE_PERIOD_SEC = 0.5         # [POSE] 출력 주기
AUTO_INTERVAL_SEC = 5.0       # auto 랜덤 이동 간격

INFLATE_RADIUS = 2
MIN_CHARGE_CELLS = 8

# follow_path 파라미터 (tune later)
V_CRUISE = 0.35
V_CHARGE = 0.8
OMEGA_TURN = 0.5
SCALE_V_MPS = 0.15
SCALE_OMEGA_RAD = 0.85


# =========================
# SHARED STATE
# =========================

stop_event = threading.Event()
state_lock = threading.Lock()

shared_state: Dict[str, Any] = {
    "mode": "auto",                 # "auto" | "manual"
    "goal_cell": None,              # 실행할 목표 (x,y)
    "pending_goal_cell": None,      # 이동 중 들어온 목표 저장용
    "is_enemy_goal": False,

    "last_cmd_ts": 0.0,
    "last_plan_ok": False,
    "last_error": "",

    # pose
    "pose_world": None,             # (x,y,theta_rad)
    "pose_cell": None,              # (cx,cy)
}


# =========================
# SERVER
# =========================

def server_poll_command() -> Optional[Dict[str, Any]]:
    """
    TODO: 서버에서 mode/goal/is_enemy_goal을 받아오는 부분
    return 예시:
      {
        "mode": "manual",
        "goal": {"type":"cell", "x":30, "y":20},
        "is_enemy_goal": True
      }
    """
    return None


# =========================
# CAMERA THREAD
# =========================

def camera_thread_main():
    """
    카메라 keep-alive 스레드.
    OpenCV/GStreamer가 실패하더라도 메인을 죽이지 않도록.
    """
    print("[CAM] thread start")
    cap = None

    while not stop_event.is_set():
        try:
            if cap is None:
                cap = cv2.VideoCapture(0)
                if not cap.isOpened():
                    print("[CAM] open failed. retry in 1s")
                    cap = None
                    time.sleep(1.0)
                    continue
                print("[CAM] opened")

            ok, _ = cap.read()
            if not ok:
                # 카메라 스트림이 깨진 경우 재시도
                time.sleep(0.2)
                cap.release()
                cap = None
                continue

            time.sleep(0.02)

        except Exception as e:
            print("[CAM] exception:", e)
            try:
                if cap is not None:
                    cap.release()
            except Exception:
                pass
            cap = None
            time.sleep(1.0)

    try:
        if cap is not None:
            cap.release()
    except Exception:
        pass
    print("[CAM] thread stop")


# =========================
# HELPERS
# =========================

def pick_random_goal_cell(m, start_cell: Tuple[int, int]) -> Optional[Tuple[int, int]]:
    max_try = 80
    for _ in range(max_try):
        gx = random.randint(0, m.width - 1)
        gy = random.randint(0, m.height - 1)
        goal = (gx, gy)
        if goal == start_cell:
            continue
        path_cells, _ = plan_path_for_goal(
            m,
            start_cell=start_cell,
            goal_cell=goal,
            is_enemy_goal=False,
            inflate_radius=INFLATE_RADIUS,
            min_charge_cells=MIN_CHARGE_CELLS,
        )
        if path_cells is not None:
            return goal
    return None


def goal_from_command(m, cmd: Dict[str, Any]) -> Tuple[Optional[Tuple[int, int]], bool, str]:
    try:
        is_enemy = bool(cmd.get("is_enemy_goal", False))
        goal = cmd.get("goal", None)
        if goal is None:
            return None, is_enemy, "no goal in command"

        gtype = goal.get("type", "cell")
        if gtype == "cell":
            gx = int(goal["x"])
            gy = int(goal["y"])
            if not (0 <= gx < m.width and 0 <= gy < m.height):
                return None, is_enemy, "goal cell out of map range"
            return (gx, gy), is_enemy, ""
        elif gtype == "world":
            wx = float(goal["x"])
            wy = float(goal["y"])
            cx, cy = world_to_cell(wx, wy, m.resolution)
            if not (0 <= cx < m.width and 0 <= cy < m.height):
                return None, is_enemy, "goal world->cell out of map range"
            return (cx, cy), is_enemy, ""
        else:
            return None, is_enemy, f"unknown goal.type={gtype}"
    except Exception as e:
        return None, False, f"goal_from_command parse error: {e}"


# =========================
# MOVE WORKER (POSE 업데이트 콜백 사용!)
# =========================

def move_worker(m, start_cell: Tuple[int, int], goal_cell: Tuple[int, int], is_enemy_goal: bool):
    """
    플랜 -> 웨이포인트 변환 -> follow_path 실행
    follow_path 내부 PoseEstimator의 실시간 업데이트를 pose_update_cb로 받아서 shared_state에 반영.
    """
    try:
        with state_lock:
            shared_state["last_error"] = ""
            shared_state["last_plan_ok"] = False

        path_cells, used_enemy_mode = plan_path_for_goal(
            m,
            start_cell=start_cell,
            goal_cell=goal_cell,
            is_enemy_goal=is_enemy_goal,
            inflate_radius=INFLATE_RADIUS,
            min_charge_cells=MIN_CHARGE_CELLS,
        )

        if path_cells is None:
            with state_lock:
                shared_state["last_plan_ok"] = False
                shared_state["last_error"] = "No path found"
            print("[MOVE] No path found")
            return
        
        for (cx, cy) in path_cells:
            print(f"  cell ({cx:2d},{cy:2d})")
        waypoints_world = [cell_to_world(x, y, m.resolution) for (x, y) in path_cells]

        with state_lock:
            shared_state["last_plan_ok"] = True

        print(f"[MOVE] start={start_cell} goal={goal_cell} enemy={is_enemy_goal} used_enemy_mode={used_enemy_mode}")
        # print(f"[MOVE] waypoints={waypoints_world}")

        # ---- 실시간 pose 업데이트 콜백 ----
        def pose_cb(px: float, py: float, ptheta: float):
            cx, cy = world_to_cell(px, py, m.resolution)
            with state_lock:
                shared_state["pose_world"] = (px, py, ptheta)
                shared_state["pose_cell"] = (cx, cy)

        # follow_path 실행 (내부에서 step_sec마다 pose_cb 호출)
        follow_path_constant_speed(
            waypoints_world=waypoints_world,
            is_enemy_goal=is_enemy_goal,
            v_cruise_norm=V_CRUISE,
            v_charge_norm=V_CHARGE,
            omega_turn_norm=OMEGA_TURN,
            scale_v_mps=SCALE_V_MPS,
            scale_omega_rad=SCALE_OMEGA_RAD,
            pose_update_cb=pose_cb,       # ✅ 여기 핵심
            step_sec=0.02,
        )

    except Exception as e:
        with state_lock:
            shared_state["last_plan_ok"] = False
            shared_state["last_error"] = f"move_worker exception: {e}"
        print("[MOVE] Exception:", e)


# =========================
# MAIN LOOP
# =========================

def main():
    # (Unicode 안전 출력)
    os.environ.setdefault("PYTHONIOENCODING", "utf-8")
    try:
        sys.stdout.reconfigure(encoding="utf-8")
    except Exception:
        pass

    print("[MAIN] Loading map ...")
    m = load_map(MAP_FILE)
    # ====== [TEST] 맵에 적 좌표가 있으면 그걸 목표로 ======
    enemy_cell = None
    if hasattr(m, "enemies") and m.enemies:
        enemy_cell = m.enemies[0]   # 첫 번째 적
    # ==========================================================

    if enemy_cell is not None:
        with state_lock:
            shared_state["mode"] = "manual"
            shared_state["pending_goal_cell"] = tuple(enemy_cell)
            shared_state["is_enemy_goal"] = True

    if m.start is None:
        raise RuntimeError("Start(S) not found in map")

    current_cell = m.start
    wx0, wy0 = cell_to_world(current_cell[0], current_cell[1], m.resolution)
    with state_lock:
        shared_state["pose_world"] = (wx0, wy0, 0.0)
        shared_state["pose_cell"] = current_cell

    cam_th = threading.Thread(target=camera_thread_main, daemon=True)
    cam_th.start()

    move_th: Optional[threading.Thread] = None
    last_auto = time.time()
    last_poll = 0.0
    last_pose_print = 0.0

    print("[MAIN] Enter loop (Ctrl+C to stop)")

    try:
        while True:
            now = time.time()
            moving = (move_th is not None) and move_th.is_alive()

            # (A) 서버 폴링 (지금은 stub)
            if now - last_poll >= SERVER_POLL_SEC:
                cmd = server_poll_command()
                last_poll = now

                if cmd is not None:
                    goal_cell_cmd, is_enemy, err = goal_from_command(m, cmd)
                    with state_lock:
                        if "mode" in cmd and cmd["mode"] in ("auto", "manual"):
                            shared_state["mode"] = cmd["mode"]
                        shared_state["is_enemy_goal"] = is_enemy
                        shared_state["last_cmd_ts"] = now
                        if err:
                            shared_state["last_error"] = err
                        if goal_cell_cmd is not None:
                            # 이동 중이면 pending으로 보관
                            if moving:
                                shared_state["pending_goal_cell"] = goal_cell_cmd
                            else:
                                shared_state["goal_cell"] = goal_cell_cmd

            # (B) mode / goal 가져오기
            with state_lock:
                mode = shared_state["mode"]
                goal_cell = shared_state["goal_cell"]
                pending_goal = shared_state["pending_goal_cell"]
                is_enemy_goal = shared_state["is_enemy_goal"]

            # (C) manual: 이동 끝났고 pending 있으면 goal로 승격
            if mode == "manual" and (not moving) and goal_cell is None and pending_goal is not None:
                with state_lock:
                    shared_state["goal_cell"] = pending_goal
                    shared_state["pending_goal_cell"] = None
                    goal_cell = pending_goal

            # (D) auto: 일정 시간마다 랜덤 목표
            if mode == "auto":
                with state_lock:
                    shared_state["pending_goal_cell"] = None

                if (not moving) and goal_cell is None and (now - last_auto) >= AUTO_INTERVAL_SEC:
                    g = pick_random_goal_cell(m, current_cell)
                    if g is not None:
                        with state_lock:
                            shared_state["goal_cell"] = g
                            shared_state["is_enemy_goal"] = False
                        goal_cell = g
                        is_enemy_goal = False
                        last_auto = now

            # (E) 이동 시작
            if (not moving) and (goal_cell is not None):
                move_th = threading.Thread(
                    target=move_worker,
                    args=(m, current_cell, goal_cell, is_enemy_goal),
                    daemon=True,
                )
                move_th.start()

                # "목표 소비" 처리: goal_cell 비우고, current_cell은 목표로 업데이트
                current_cell = goal_cell
                with state_lock:
                    shared_state["goal_cell"] = None

            # (F) 주기적으로 [POSE] 출력 (STATUS 대신)
            if now - last_pose_print >= POSE_PERIOD_SEC:
                with state_lock:
                    pose_world = shared_state["pose_world"]
                    pose_cell = shared_state["pose_cell"]
                if pose_world is not None and pose_cell is not None:
                    px, py, pth = pose_world
                    cx, cy = pose_cell
                    print(f"[POSE] world=({px:.2f},{py:.2f},theta_deg={math.degrees(pth):.1f}) cell=({cx},{cy})")
                else:
                    print("[POSE] None")
                last_pose_print = now

            time.sleep(0.01)

    except KeyboardInterrupt:
        print("[MAIN] KeyboardInterrupt -> stopping")
    finally:
        stop_event.set()
        time.sleep(0.2)
        print("[MAIN] Exit")


if __name__ == "__main__":
    main()
