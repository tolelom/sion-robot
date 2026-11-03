"""격자 경로 후처리.

line_of_sight와 bresenham_line은 오차 누적 방식이 달라 같은 두 점에서도 지나는 셀이 다르다.
합치면 평활화·돌진 판정 결과가 바뀌므로 따로 둔다.
"""
import math
from typing import List, Tuple

from config import MIN_CHARGE_CELLS
from map_loader import CellType, MapData

def line_of_sight(m: MapData,
                  a: Tuple[int, int],
                  b: Tuple[int, int]) -> bool:
    x0, y0 = a
    x1, y1 = b

    dx = x1 - x0
    dy = y1 - y0

    sx = 1 if dx > 0 else -1
    sy = 1 if dy > 0 else -1

    dx = abs(dx)
    dy = abs(dy)

    x, y = x0, y0

    if dx >= dy:
        err = dx / 2
        while x != x1:
            if m.grid[y][x] == CellType.WALL:
                return False
            err -= dy
            if err < 0:
                y += sy
                err += dx
            x += sx
    else:
        err = dy / 2
        while y != y1:
            if m.grid[y][x] == CellType.WALL:
                return False
            err -= dx
            if err < 0:
                x += sx
                err += dy
            y += sy

    if m.grid[y1][x1] == CellType.WALL:
        return False

    return True


def _farthest_visible_index(m: MapData, path: List[Tuple[int, int]], i: int) -> int:
    """바로 다음 셀이 안 보여도 최소 한 칸은 전진한다."""
    j = i + 1
    while j < len(path) and line_of_sight(m, path[i], path[j]):
        j += 1
    return max(j - 1, i + 1)


def smooth_path_cells(m: MapData,
                      path: List[Tuple[int, int]]) -> List[Tuple[int, int]]:
    if not path:
        return path

    if len(path) <= 2:
        return path[:]

    smoothed = [path[0]]
    i = 0
    while i < len(path) - 1:
        i = _farthest_visible_index(m, path, i)
        smoothed.append(path[i])

    if len(smoothed) >= 2 and smoothed[-1] == smoothed[-2]:
        smoothed.pop()

    return smoothed


def inflate_map(m: MapData, radius_cells: int) -> MapData:
    """벽을 radius_cells만큼 두껍게 만든 새 맵. 적·출발 셀은 벽에 덮이지 않게 남긴다."""
    W, H = m.width, m.height
    new_grid = [[CellType.EMPTY for _ in range(W)] for _ in range(H)]

    for y in range(H):
        for x in range(W):
            if m.grid[y][x] == CellType.WALL:
                for yy in range(max(0, y - radius_cells), min(H, y + radius_cells + 1)):
                    for xx in range(max(0, x - radius_cells), min(W, x + radius_cells + 1)):
                        new_grid[yy][xx] = CellType.WALL

    for (ex, ey) in m.enemies:
        new_grid[ey][ex] = CellType.ENEMY

    if m.start is not None:
        sx, sy = m.start
        if new_grid[sy][sx] == CellType.WALL:
            new_grid[sy][sx] = CellType.EMPTY

    return MapData(
        width=m.width,
        height=m.height,
        resolution=m.resolution,
        grid=new_grid,
        start=m.start,
        enemies=m.enemies[:],
    )


def bresenham_line(a: Tuple[int, int],
                   b: Tuple[int, int]) -> List[Tuple[int, int]]:
    x0, y0 = a
    x1, y1 = b
    points: List[Tuple[int, int]] = []

    dx = abs(x1 - x0)
    dy = -abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx + dy

    x, y = x0, y0
    while True:
        points.append((x, y))
        if x == x1 and y == y1:
            break
        e2 = 2 * err
        if e2 >= dy:
            err += dy
            x += sx
        if e2 <= dx:
            err += dx
            y += sy

    return points


def attach_charge_segment(
    m_orig: MapData,
    smooth_path: List[Tuple[int, int]],
    min_charge_cells: int = MIN_CHARGE_CELLS,
) -> List[Tuple[int, int]]:
    """목표가 직선으로 보이는 가장 먼 웨이포인트부터 목표까지를 돌진 구간으로 바꾼다.

    제어용 웨이포인트라 돌진 구간은 [돌진 시작점, 목표] 두 점만 남긴다.
    """
    if len(smooth_path) < 2:
        return smooth_path

    goal = smooth_path[-1]
    charge_start_idx = None
    best_len = 0.0

    for i in range(len(smooth_path) - 2, -1, -1):
        p = smooth_path[i]
        if not line_of_sight(m_orig, p, goal):
            continue

        dist = math.hypot(goal[0] - p[0], goal[1] - p[1])
        if dist >= min_charge_cells and dist > best_len:
            best_len = dist
            charge_start_idx = i

    if charge_start_idx is None:
        return smooth_path

    return smooth_path[:charge_start_idx + 1] + [goal]

