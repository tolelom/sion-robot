"""
A* 경로 탐색 구현 (로봇 측, Python).

이 모듈은 로봇이 직접 사용하는 경로 계획기다.
- astar_normal: 8방향 최단 경로 (유클리드 휴리스틱)
- astar_enemy : 같은 길이 중 회전 횟수가 적은 경로 우선 (적 추적용)
- plan_path   : 목표 셀이 ENEMY면 astar_enemy, 그 외엔 astar_normal

경계 조건:
  - start == goal → [start] 반환
  - goal이 장애물 → None 반환
  - 경로 없음 → None 반환

관련 구현 (의도적으로 분리되어 있음):
  - sion-api/algorithms/astar.go
      웹 시뮬레이터/대시보드용 (POST /api/pathfinding). 일반 A*만 제공한다.
      웹 API는 셀 타입(ENEMY/OBSTACLE 등)을 전달하지 않으므로 회전 최소화 변형을
      이식할 필요가 없다. 휴리스틱·이동 비용은 본 모듈과 동일하게 유지한다.
  - sion-web
      자체 A*를 구현하지 않고 백엔드 API에 위임한다.

본 모듈의 알고리즘(휴리스틱, 8방향 비용)을 바꿀 때는 sion-api의
일반 A*와 결과가 일치하도록 같이 손볼 것.
"""
import heapq
import math
from dataclasses import dataclass
from typing import Callable, Dict, Iterator, List, Optional, Tuple

from map_loader import CellType, MapData

Cell = Tuple[int, int]


# 8방향 이동 (dx, dy) 및 이동 비용
DIRS_8 = [
    (1, 0, 1.0),   # 0: 동
    (-1, 0, 1.0),  # 1: 서
    (0, 1, 1.0),   # 2: 북
    (0, -1, 1.0),  # 3: 남
    (1, 1, math.sqrt(2.0)),    # 4: 북동
    (-1, 1, math.sqrt(2.0)),   # 5: 북서
    (1, -1, math.sqrt(2.0)),   # 6: 남동
    (-1, -1, math.sqrt(2.0)),  # 7: 남서
]


INF = 1e18
_MAX_TURNS = 10**9          # 회전 수 비교용 '무한대'
_COST_EPS = 1e-6            # 거리 동률 판정 허용 오차


def _euclidean_to(goal: Cell) -> Callable[[int, int], float]:
    """goal까지의 유클리드 거리 휴리스틱 함수를 만든다."""
    gx, gy = goal

    def h(x: int, y: int) -> float:
        return math.hypot(x - gx, y - gy)

    return h


def _neighbors(m: MapData, x: int, y: int) -> Iterator[Tuple[int, int, int, float]]:
    """(x, y)에서 갈 수 있는 8방향 이웃을 (방향 인덱스, nx, ny, 비용)으로 낸다. 맵 밖·벽은 제외."""
    for d, (dx, dy, cost) in enumerate(DIRS_8):
        nx, ny = x + dx, y + dy
        if not (0 <= nx < m.width and 0 <= ny < m.height):
            continue
        if m.grid[ny][nx] == CellType.WALL:
            continue
        yield d, nx, ny, cost


# -----------------------
# 1) 일반 A* (최단 거리만)
# -----------------------
def astar_normal(
    m: MapData,
    start: Tuple[int, int],
    goal: Tuple[int, int],
) -> Optional[List[Tuple[int, int]]]:
    """
    8방향 A* - 단순 최단 거리
    """
    sx, sy = start
    gx, gy = goal
    W, H = m.width, m.height
    h = _euclidean_to(goal)

    # g값 테이블
    dist = [[INF] * W for _ in range(H)]
    parent: List[List[Optional[Tuple[int, int]]]] = [
        [None] * W for _ in range(H)
    ]

    pq: List[Tuple[float, float, int, int]] = []
    dist[sy][sx] = 0.0
    heapq.heappush(pq, (h(sx, sy), 0.0, sx, sy))

    while pq:
        f, g, x, y = heapq.heappop(pq)
        if g > dist[y][x]:
            continue

        if (x, y) == (gx, gy):
            break

        for _, nx, ny, cost in _neighbors(m, x, y):
            ng = g + cost
            if ng < dist[ny][nx]:
                dist[ny][nx] = ng
                parent[ny][nx] = (x, y)
                nf = ng + h(nx, ny)
                heapq.heappush(pq, (nf, ng, nx, ny))

    if dist[gy][gx] >= INF:
        return None

    # 경로 복원
    path: List[Tuple[int, int]] = []
    x, y = gx, gy
    while (x, y) != (sx, sy):
        path.append((x, y))
        px, py = parent[y][x]
        x, y = px, py
    path.append((sx, sy))
    path.reverse()
    return path


# -------------------------------
# 2) 적(ENEMY)용 A*: 회전 최소화
# -------------------------------

@dataclass(frozen=True, eq=True)
class _SearchState:
    """적 A* 탐색 상태: 위치 + 마지막 이동 방향(회전 수를 세기 위해)."""
    x: int
    y: int
    d: int  # DIRS_8 인덱스, 시작은 -1


def astar_enemy(
    m: MapData,
    start: Tuple[int, int],
    goal: Tuple[int, int],
    lambda_turn: float = 0.05,
) -> Optional[List[Tuple[int, int]]]:
    """
    ENEMY 목표용 A*.
    - 8방향 이동
    - 거리(총 길이)를 최소화
    - 같은 길이 중에서는 '회전(방향 변경) 횟수'가 적은 경로 우선
      -> 직선 구간이 더 길어지는 효과
    """
    sx, sy = start
    gx, gy = goal
    h = _euclidean_to(goal)

    # dist[state] = (g, turns)
    dist: Dict[_SearchState, Tuple[float, int]] = {}
    parent: Dict[_SearchState, _SearchState] = {}

    start_state = _SearchState(sx, sy, -1)
    dist[start_state] = (0.0, 0)

    pq: List[Tuple[float, float, int, int, _SearchState]] = []
    counter = 0  # 우선순위 큐에서 State 비교를 피하기 위한 tie-breaker

    # (f, g, turns, counter, state)
    heapq.heappush(pq, (h(sx, sy), 0.0, 0, counter, start_state))
    counter += 1

    best_goal: Optional[_SearchState] = None

    while pq:
        f, g, tcnt, _, s = heapq.heappop(pq)
        if (g, tcnt) != dist.get(s, (INF, _MAX_TURNS)):
            continue

        if (s.x, s.y) == (gx, gy):
            best_goal = s
            break

        for nd, nx, ny, cost in _neighbors(m, s.x, s.y):
            ng = g + cost
            nturn = tcnt + (0 if s.d == -1 or s.d == nd else 1)
            ns = _SearchState(nx, ny, nd)

            prev = dist.get(ns, (INF, _MAX_TURNS))
            # 우선 g(거리) 최소, 동률이면 회전 수 최소
            if (ng < prev[0]) or (abs(ng - prev[0]) < _COST_EPS and nturn < prev[1]):
                dist[ns] = (ng, nturn)
                parent[ns] = s
                nf = ng + h(nx, ny) + lambda_turn * nturn
                heapq.heappush(pq, (nf, ng, nturn, counter, ns))
                counter += 1

    if best_goal is None:
        return None

    # 경로 복원 (x, y만)
    path: List[Tuple[int, int]] = []
    s = best_goal
    while True:
        path.append((s.x, s.y))
        if (s.x, s.y) == (sx, sy) and s.d == -1:
            break
        s = parent[s]
    path.reverse()
    return path


# -----------------------
# 유틸: 목표 타입 따라 A*
# -----------------------
def plan_path(
    m: MapData,
    start: Tuple[int, int],
    goal: Tuple[int, int],
) -> Optional[List[Tuple[int, int]]]:
    """
    goal 셀 타입에 따라 적절한 A* 선택
    - ENEMY면 astar_enemy
    - 아니면 astar_normal
    """
    gx, gy = goal
    if m.grid[gy][gx] == CellType.ENEMY:
        return astar_enemy(m, start, goal)
    else:
        return astar_normal(m, start, goal)
