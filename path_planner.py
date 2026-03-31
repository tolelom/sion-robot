# path_planner.py — 경로 계획 파이프라인

from typing import List, Tuple, Optional

from map_loader import MapData
from astar import astar_normal, astar_enemy
from path_utils import smooth_path_cells, inflate_map, attach_charge_segment
from config import INFLATE_RADIUS, MIN_CHARGE_CELLS


def plan_path_for_goal(
    m_orig: MapData,
    start_cell: Tuple[int, int],
    goal_cell: Tuple[int, int],
    is_enemy_goal: bool,
    inflate_radius: int = INFLATE_RADIUS,
    min_charge_cells: int = MIN_CHARGE_CELLS,
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
