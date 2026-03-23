# path_utils.py

import math
from typing import List, Tuple

from map_loader import MapData, CellType


# ----------------------------------------------------
# 1. a->b 직선에 장애물이 끼는지 체크 (이미 있었던 코드)
# ----------------------------------------------------
def line_of_sight(m: MapData,
                  a: Tuple[int, int],
                  b: Tuple[int, int]) -> bool:
    """
    격자에서 a->b 직선 경로에 WALL이 하나라도 끼는지 검사.
    Bresenham 기반 간단한 라인 트레이싱.
    a, b는 (x, y) 셀 인덱스.
    """
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

    # 마지막 점도 검사
    if m.grid[y1][x1] == CellType.WALL:
        return False

    return True


# ----------------------------------------------------
# 2. A* 경로를 라인오브사이트 기반으로 스무딩
# ----------------------------------------------------
def smooth_path_cells(m: MapData,
                      path: List[Tuple[int, int]]) -> List[Tuple[int, int]]:
    """
    A*가 만든 셀 경로를 라인오브사이트 기반으로 단순화.
    - 입력: [(x0,y0), (x1,y1), ..., (xn,yn)]
    - 출력: 불필요한 중간 노드를 제거한 더 매끈한 경로
    """
    if not path:
        return path

    if len(path) <= 2:
        return path[:]

    smoothed: List[Tuple[int, int]] = []
    i = 0
    n = len(path)

    smoothed.append(path[0])

    while i < n - 1:
        j = i + 1
        # i에서 j까지 직선으로 갈 수 있는 한 최대한 멀리 확장
        while j < n and line_of_sight(m, path[i], path[j]):
            j += 1
        # line_of_sight가 깨진 직전까지가 유효 → j-1이 새 웨이포인트
        next_idx = j - 1
        if next_idx == i:  # 안전 체크
            next_idx = i + 1
        smoothed.append(path[next_idx])
        i = next_idx

    # 혹시 마지막 점이 중복되면 정리
    if len(smoothed) >= 2 and smoothed[-1] == smoothed[-2]:
        smoothed.pop()

    return smoothed


# ----------------------------------------------------
# 3. 장애물 인플레이션 (로봇 크기 반영)
# ----------------------------------------------------
def inflate_map(m: MapData, radius_cells: int) -> MapData:
    """
    장애물을 radius_cells 만큼 두껍게 부풀린 새로운 MapData를 반환.
    - 원본 m은 건드리지 않음
    - start / enemies 좌표는 그대로 복사
    """
    W, H = m.width, m.height
    new_grid = [[CellType.EMPTY for _ in range(W)] for _ in range(H)]

    # 원본 WALL 기준으로 주변을 WALL로 채움
    for y in range(H):
        for x in range(W):
            if m.grid[y][x] == CellType.WALL:
                for yy in range(max(0, y - radius_cells), min(H, y + radius_cells + 1)):
                    for xx in range(max(0, x - radius_cells), min(W, x + radius_cells + 1)):
                        new_grid[yy][xx] = CellType.WALL

    # ENEMY 위치는 벽 위에 있더라도 ENEMY로 설정
    for (ex, ey) in m.enemies:
        new_grid[ey][ex] = CellType.ENEMY

    # START 위치도 확보
    if m.start is not None:
        sx, sy = m.start
        if new_grid[sy][sx] == CellType.WALL:
            new_grid[sy][sx] = CellType.EMPTY

    inflated = MapData(
        width=m.width,
        height=m.height,
        resolution=m.resolution,
        grid=new_grid,
        start=m.start,
        enemies=m.enemies[:],
    )
    return inflated


# ----------------------------------------------------
# 4. 직선 셀 시퀀스 생성 (Bresenham)
# ----------------------------------------------------
def bresenham_line(a: Tuple[int, int],
                   b: Tuple[int, int]) -> List[Tuple[int, int]]:
    """격자에서 a->b 직선상의 셀들을 반환 (Bresenham)"""
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


# ----------------------------------------------------
# 5. 적(돌진)용: 마지막 구간을 직선 돌진으로 교체
# ----------------------------------------------------
def attach_charge_segment(
    m_orig: MapData,
    smooth_path: List[Tuple[int, int]],
    min_charge_cells: int = 8,   # 직선 돌진 최소 길이 (셀 단위)
) -> List[Tuple[int, int]]:
    """
    스무딩된 경로의 마지막 부분을 'goal로 향하는 직선' 구간으로 교체.
    - 여기서는 '제어용 웨이포인트' 관점에서:
      → 돌진 구간은 [charge_start, goal] 딱 두 점만 남긴다.
    - 실제로 직선 상 경로를 그리고 싶으면,
      출력/디버깅 시에만 Bresenham으로 채워서 보여주면 된다.
    """
    if len(smooth_path) < 2:
        return smooth_path

    goal = smooth_path[-1]

    # 뒤에서부터 거슬러 올라가며 가장 먼 line-of-sight 가능 지점 찾기
    best_idx = len(smooth_path) - 2
    best_len = 0.0

    for i in range(len(smooth_path) - 2, -1, -1):
        p = smooth_path[i]
        # 장애물에 안 부딪히는 직선인가?
        if not line_of_sight(m_orig, p, goal):
            continue

        dx = goal[0] - p[0]
        dy = goal[1] - p[1]
        dist = math.hypot(dx, dy)

        if dist >= min_charge_cells and dist > best_len:
            best_len = dist
            best_idx = i

    # 조건 만족하는 게 없으면 그냥 원래 경로 사용
    if best_len <= 0.0:
        return smooth_path

    charge_start = smooth_path[best_idx]

    # ★ 핵심:
    # - best_idx 이전까지는 그대로 둠
    # - 돌진 구간은 [charge_start, goal] 두 점만 남김
    new_path = smooth_path[:best_idx + 1] + [goal]
    return new_path

