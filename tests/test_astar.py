"""A* 경로 탐색 단위 테스트.

대상: astar_normal, astar_enemy, plan_path (8방향, 회전 최소화 변형 포함).
경계 조건: start==goal, 장애물에 가로막힘, goal이 벽, 8방향 이동 가능 여부.
"""
import math

import pytest

from astar import DIRS_8, astar_enemy, astar_normal, plan_path
from map_loader import CellType
from tests.conftest import make_map


# ---------------------------------------------------------------------------
# astar_normal
# ---------------------------------------------------------------------------

class TestAStarNormal:
    def test_start_equals_goal_returns_single_cell(self):
        m = make_map(
            """
            . . .
            . . .
            . . .
            """
        )
        path = astar_normal(m, (1, 1), (1, 1))
        assert path == [(1, 1)]

    def test_straight_line_no_obstacle(self):
        m = make_map(
            """
            . . . . .
            . . . . .
            . . . . .
            """
        )
        path = astar_normal(m, (0, 1), (4, 1))
        assert path is not None
        assert path[0] == (0, 1)
        assert path[-1] == (4, 1)
        # 5개 셀: start 포함, 4번 이동
        assert len(path) == 5

    def test_diagonal_movement_uses_8_directions(self):
        m = make_map(
            """
            . . . .
            . . . .
            . . . .
            . . . .
            """
        )
        path = astar_normal(m, (0, 0), (3, 3))
        assert path is not None
        # 8방향이면 (3,3)까지 4셀 (start 포함). 4방향이면 7셀.
        assert len(path) == 4

    def test_path_goes_around_wall(self):
        # 위에서 4번째 줄까지 거의 막혀 있고, 맨 오른쪽 한 칸만 통로.
        m = make_map(
            """
            . . . . .
            # # # # .
            . . . . .
            . # # # #
            . . . . .
            """
        )
        path = astar_normal(m, (0, 0), (0, 4))
        assert path is not None
        assert path[0] == (0, 0)
        assert path[-1] == (0, 4)
        # 벽 셀을 지나지 않음
        for x, y in path:
            assert m.grid[y][x] != CellType.WALL
        # 우회하느라 직선 거리(5)보다 길어야 함
        assert len(path) > 5

    def test_unreachable_returns_none(self):
        m = make_map(
            """
            . # . . .
            . # . . .
            . # . . .
            . # . . .
            . # . . .
            """
        )
        # 왼쪽 컬럼은 벽으로 막혀서 오른쪽으로 못 감
        path = astar_normal(m, (0, 0), (4, 4))
        assert path is None

    def test_goal_on_wall_returns_none(self):
        m = make_map(
            """
            . . . . .
            . # # # .
            . . . . .
            """
        )
        path = astar_normal(m, (0, 0), (2, 1))  # 벽 위
        assert path is None

    def test_path_only_uses_valid_steps(self):
        m = make_map(
            """
            . . . . .
            . . . . .
            . . . . .
            """
        )
        path = astar_normal(m, (0, 0), (4, 2))
        assert path is not None
        valid_steps = {(d[0], d[1]) for d in DIRS_8}
        for prev, curr in zip(path, path[1:]):
            step = (curr[0] - prev[0], curr[1] - prev[1])
            assert step in valid_steps, f"잘못된 이동: {prev} -> {curr}"


# ---------------------------------------------------------------------------
# astar_enemy
# ---------------------------------------------------------------------------

class TestAStarEnemy:
    def test_straight_line_prefers_no_turn(self):
        m = make_map(
            """
            . . . . . . .
            . . . . . . .
            . . . . . . .
            """
        )
        path = astar_enemy(m, (0, 1), (6, 1))
        assert path is not None
        # 회전 없이 일직선이므로 방향 한 번도 안 바뀜
        directions = set()
        for prev, curr in zip(path, path[1:]):
            directions.add((curr[0] - prev[0], curr[1] - prev[1]))
        assert len(directions) == 1

    def test_returns_none_when_unreachable(self):
        m = make_map(
            """
            . # .
            . # .
            . # .
            """
        )
        path = astar_enemy(m, (0, 0), (2, 2))
        assert path is None

    def test_start_equals_goal(self):
        m = make_map(
            """
            . . .
            . . .
            . . .
            """
        )
        path = astar_enemy(m, (1, 1), (1, 1))
        assert path == [(1, 1)]

    def test_distance_matches_normal_when_no_turn_constraint_helps(self):
        # 같은 맵에서 enemy A*도 최소 거리는 normal A*와 동일 길이여야 한다.
        m = make_map(
            """
            . . . . .
            . . . . .
            . . . . .
            . . . . .
            """
        )
        normal_path = astar_normal(m, (0, 0), (4, 3))
        enemy_path = astar_enemy(m, (0, 0), (4, 3))
        assert normal_path is not None and enemy_path is not None

        def length(p):
            total = 0.0
            for a, b in zip(p, p[1:]):
                total += math.hypot(b[0] - a[0], b[1] - a[1])
            return total

        # enemy 변형은 거리 동률 중 회전 최소를 고름 → 거리는 같거나 매우 근접
        assert abs(length(normal_path) - length(enemy_path)) < 1e-6


# ---------------------------------------------------------------------------
# plan_path: goal cell 타입에 따른 디스패치
# ---------------------------------------------------------------------------

class TestPlanPath:
    def test_dispatches_to_enemy_when_goal_is_enemy_cell(self):
        m = make_map(
            """
            . . . . .
            . . . . .
            . . . E .
            """
        )
        path = plan_path(m, (0, 0), (3, 2))
        assert path is not None
        assert path[-1] == (3, 2)

    def test_dispatches_to_normal_when_goal_is_empty(self):
        m = make_map(
            """
            . . . . .
            . . . . .
            . . . . .
            """
        )
        path = plan_path(m, (0, 0), (4, 2))
        assert path is not None
        assert path[0] == (0, 0)
        assert path[-1] == (4, 2)
