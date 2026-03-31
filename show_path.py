# show_path.py — 경로 시각화 유틸리티

import json
import logging
from typing import List, Tuple

from map_loader import load_map, cell_to_world
from path_planner import plan_path_for_goal
from path_utils import bresenham_line
from config import INFLATE_RADIUS, MIN_CHARGE_CELLS

logger = logging.getLogger(__name__)


def print_map_with_path(
    cells: List[str],
    path: List[Tuple[int, int]],
    title: str,
) -> None:
    logger.info("\n%s", title)
    char_grid = [list(row) for row in cells]

    if path:
        for i in range(len(path) - 1):
            (x0, y0) = path[i]
            (x1, y1) = path[i + 1]
            seg = bresenham_line((x0, y0), (x1, y1))
            for (sx, sy) in seg:
                if char_grid[sy][sx] in (".",):
                    char_grid[sy][sx] = "*"

        sx, sy = path[0]
        gx, gy = path[-1]
        char_grid[sy][sx] = "S"
        char_grid[gy][gx] = "E"

    for row in reversed(char_grid):
        logger.info("".join(row))


def main():
    logging.basicConfig(
        level=logging.INFO,
        format="%(message)s",
    )

    m_orig = load_map("map_smallroom_60x60.json")
    if m_orig.start is None:
        raise RuntimeError("Start(S) not found in map")

    start = m_orig.start

    if not m_orig.enemies:
        raise RuntimeError("Enemy(E) not found in map")
    goal_cell = m_orig.enemies[0]
    is_enemy_goal = False

    logger.info("Start cell: %s", start)
    logger.info("Goal  cell: %s", goal_cell)
    logger.info("is_enemy_goal: %s", is_enemy_goal)

    path, used_enemy_mode = plan_path_for_goal(
        m_orig,
        start_cell=start,
        goal_cell=goal_cell,
        is_enemy_goal=is_enemy_goal,
        inflate_radius=INFLATE_RADIUS,
        min_charge_cells=MIN_CHARGE_CELLS,
    )

    if path is None:
        logger.info("No path found")
        return

    logger.info("Used enemy mode: %s", used_enemy_mode)
    logger.info("Final path length (cells): %d", len(path))

    with open("map_smallroom_60x60.json", "r") as f:
        raw = json.load(f)
        cells = raw["cells"]

    print_map_with_path(cells, path, "Final path (with charge if enemy)")

    logger.info("\nFinal path (world coordinates, meters):")
    for (cx, cy) in path:
        wx, wy = cell_to_world(cx, cy, m_orig.resolution)
        logger.info("  cell (%2d,%2d) -> world (%5.2f, %5.2f)", cx, cy, wx, wy)


if __name__ == "__main__":
    main()
