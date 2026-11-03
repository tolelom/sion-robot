"""로봇 측 A* 경로 계획기.

sion-api/algorithms/astar.go(웹 시뮬레이터용 일반 A*)와 휴리스틱·8방향 비용이 같아야 한다.
웹 API는 셀 타입을 넘기지 않아 회전 최소화 변형(astar_enemy)은 이식하지 않았다.
"""
import heapq
import itertools
import math
from dataclasses import dataclass
from typing import Callable, Dict, Iterator, List, Optional, Tuple

from map_loader import CellType, MapData

Cell = Tuple[int, int]

_DIAGONAL_COST = math.sqrt(2.0)
DIRS_8 = [
    (1, 0, 1.0),
    (-1, 0, 1.0),
    (0, 1, 1.0),
    (0, -1, 1.0),
    (1, 1, _DIAGONAL_COST),
    (-1, 1, _DIAGONAL_COST),
    (1, -1, _DIAGONAL_COST),
    (-1, -1, _DIAGONAL_COST),
]


INF = 1e18
_INF_TURNS = 10**9
_COST_TIE_EPS = 1e-6
_NO_DIRECTION = -1


def _euclidean_to(goal: Cell) -> Callable[[int, int], float]:
    gx, gy = goal

    def h(x: int, y: int) -> float:
        return math.hypot(x - gx, y - gy)

    return h


def _neighbors(m: MapData, x: int, y: int) -> Iterator[Tuple[int, int, int, float]]:
    """(DIRS_8 인덱스, nx, ny, 비용)을 낸다."""
    for d, (dx, dy, cost) in enumerate(DIRS_8):
        nx, ny = x + dx, y + dy
        if not (0 <= nx < m.width and 0 <= ny < m.height):
            continue
        if m.grid[ny][nx] == CellType.WALL:
            continue
        yield d, nx, ny, cost


def astar_normal(
    m: MapData,
    start: Tuple[int, int],
    goal: Tuple[int, int],
) -> Optional[List[Tuple[int, int]]]:
    sx, sy = start
    gx, gy = goal
    W, H = m.width, m.height
    h = _euclidean_to(goal)

    g_cost = [[INF] * W for _ in range(H)]
    parent: List[List[Optional[Tuple[int, int]]]] = [
        [None] * W for _ in range(H)
    ]

    pq: List[Tuple[float, float, int, int]] = []
    g_cost[sy][sx] = 0.0
    heapq.heappush(pq, (h(sx, sy), 0.0, sx, sy))

    while pq:
        f, g, x, y = heapq.heappop(pq)
        if g > g_cost[y][x]:
            continue

        if (x, y) == (gx, gy):
            break

        for _, nx, ny, cost in _neighbors(m, x, y):
            ng = g + cost
            if ng < g_cost[ny][nx]:
                g_cost[ny][nx] = ng
                parent[ny][nx] = (x, y)
                nf = ng + h(nx, ny)
                heapq.heappush(pq, (nf, ng, nx, ny))

    if g_cost[gy][gx] >= INF:
        return None

    path: List[Tuple[int, int]] = []
    x, y = gx, gy
    while (x, y) != (sx, sy):
        path.append((x, y))
        px, py = parent[y][x]
        x, y = px, py
    path.append((sx, sy))
    path.reverse()
    return path


@dataclass(frozen=True, eq=True)
class _SearchState:
    x: int
    y: int
    last_dir: int


def _is_better(g: float, turns: int, best: Tuple[float, int]) -> bool:
    best_g, best_turns = best
    return g < best_g or (abs(g - best_g) < _COST_TIE_EPS and turns < best_turns)


def astar_enemy(
    m: MapData,
    start: Tuple[int, int],
    goal: Tuple[int, int],
    lambda_turn: float = 0.05,
) -> Optional[List[Tuple[int, int]]]:
    """최단 거리 경로 중 회전이 가장 적은(직선 구간이 긴) 경로를 찾는다."""
    sx, sy = start
    gx, gy = goal
    h = _euclidean_to(goal)

    best_cost_turns: Dict[_SearchState, Tuple[float, int]] = {}
    parent: Dict[_SearchState, _SearchState] = {}

    start_state = _SearchState(sx, sy, _NO_DIRECTION)
    best_cost_turns[start_state] = (0.0, 0)

    tie_breaker = itertools.count()
    pq: List[Tuple[float, float, int, int, _SearchState]] = []
    heapq.heappush(pq, (h(sx, sy), 0.0, 0, next(tie_breaker), start_state))

    best_goal: Optional[_SearchState] = None

    while pq:
        f, g, turns, _, s = heapq.heappop(pq)
        if (g, turns) != best_cost_turns.get(s, (INF, _INF_TURNS)):
            continue

        if (s.x, s.y) == (gx, gy):
            best_goal = s
            break

        for nd, nx, ny, cost in _neighbors(m, s.x, s.y):
            ng = g + cost
            is_turn = s.last_dir not in (_NO_DIRECTION, nd)
            nturns = turns + (1 if is_turn else 0)
            ns = _SearchState(nx, ny, nd)

            if _is_better(ng, nturns, best_cost_turns.get(ns, (INF, _INF_TURNS))):
                best_cost_turns[ns] = (ng, nturns)
                parent[ns] = s
                nf = ng + h(nx, ny) + lambda_turn * nturns
                heapq.heappush(pq, (nf, ng, nturns, next(tie_breaker), ns))

    if best_goal is None:
        return None

    path: List[Tuple[int, int]] = []
    s = best_goal
    while True:
        path.append((s.x, s.y))
        if s == start_state:
            break
        s = parent[s]
    path.reverse()
    return path


def plan_path(
    m: MapData,
    start: Tuple[int, int],
    goal: Tuple[int, int],
) -> Optional[List[Tuple[int, int]]]:
    gx, gy = goal
    if m.grid[gy][gx] == CellType.ENEMY:
        return astar_enemy(m, start, goal)
    else:
        return astar_normal(m, start, goal)
