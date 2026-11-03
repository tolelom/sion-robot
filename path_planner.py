from typing import List, Tuple, Optional

from map_loader import MapData
from astar import astar_normal, astar_enemy
from path_utils import smooth_path_cells, inflate_map, attach_charge_segment
from config import INFLATE_RADIUS_CELLS, MIN_CHARGE_CELLS


def plan_path_for_goal(
    m_orig: MapData,
    start_cell: Tuple[int, int],
    goal_cell: Tuple[int, int],
    is_enemy_goal: bool,
    inflate_radius: int = INFLATE_RADIUS_CELLS,
    min_charge_cells: int = MIN_CHARGE_CELLS,
) -> Tuple[Optional[List[Tuple[int, int]]], bool]:
    m = inflate_map(m_orig, radius_cells=inflate_radius)

    if is_enemy_goal:
        raw_path = astar_enemy(m, start_cell, goal_cell)
    else:
        raw_path = astar_normal(m, start_cell, goal_cell)

    if raw_path is None:
        return None, is_enemy_goal

    smooth_path = smooth_path_cells(m, raw_path)

    if is_enemy_goal:
        final_path = attach_charge_segment(
            m_orig, smooth_path, min_charge_cells=min_charge_cells
        )
    else:
        final_path = smooth_path

    return final_path, is_enemy_goal
