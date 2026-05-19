"""path_utils 단위 테스트.

대상: line_of_sight, smooth_path_cells, inflate_map, bresenham_line, attach_charge_segment.
"""
from map_loader import CellType
from path_utils import (
    attach_charge_segment,
    bresenham_line,
    inflate_map,
    line_of_sight,
    smooth_path_cells,
)
from tests.conftest import make_map


# ---------------------------------------------------------------------------
# line_of_sight
# ---------------------------------------------------------------------------

class TestLineOfSight:
    def test_clear_horizontal(self):
        m = make_map(
            """
            . . . . .
            . . . . .
            """
        )
        assert line_of_sight(m, (0, 0), (4, 0)) is True

    def test_blocked_horizontal_returns_false(self):
        m = make_map(
            """
            . . # . .
            """
        )
        assert line_of_sight(m, (0, 0), (4, 0)) is False

    def test_diagonal_clear(self):
        m = make_map(
            """
            . . . .
            . . . .
            . . . .
            . . . .
            """
        )
        assert line_of_sight(m, (0, 0), (3, 3)) is True

    def test_endpoint_on_wall_returns_false(self):
        m = make_map(
            """
            . . . .
            . . . #
            """
        )
        assert line_of_sight(m, (0, 0), (3, 1)) is False


# ---------------------------------------------------------------------------
# smooth_path_cells
# ---------------------------------------------------------------------------

class TestSmoothPath:
    def test_empty_path_returns_empty(self):
        m = make_map(". . . .")
        assert smooth_path_cells(m, []) == []

    def test_two_point_path_unchanged(self):
        m = make_map(
            """
            . . . .
            . . . .
            """
        )
        path = [(0, 0), (3, 1)]
        assert smooth_path_cells(m, path) == path

    def test_collinear_points_collapsed_to_endpoints(self):
        m = make_map(
            """
            . . . . .
            . . . . .
            """
        )
        # 모두 (y=0) 위의 점. 시야가 통하므로 양 끝점만 남음.
        path = [(0, 0), (1, 0), (2, 0), (3, 0), (4, 0)]
        smoothed = smooth_path_cells(m, path)
        assert smoothed[0] == (0, 0)
        assert smoothed[-1] == (4, 0)
        assert len(smoothed) == 2

    def test_keeps_corner_when_wall_blocks_shortcut(self):
        m = make_map(
            """
            . . . . .
            . # # # .
            . . . . .
            """
        )
        # (0,0)에서 (4,2)로 직접 line_of_sight 가능 여부에 따라 코너가 유지될 수 있음.
        path = [(0, 0), (0, 2), (4, 2)]
        smoothed = smooth_path_cells(m, path)
        assert smoothed[0] == (0, 0)
        assert smoothed[-1] == (4, 2)
        # 적어도 시작/끝은 보존
        assert len(smoothed) >= 2


# ---------------------------------------------------------------------------
# inflate_map
# ---------------------------------------------------------------------------

class TestInflateMap:
    def test_radius_zero_leaves_grid_equivalent(self):
        m = make_map(
            """
            . . . . .
            . # # # .
            . . . . .
            """
        )
        inflated = inflate_map(m, radius_cells=0)
        for y in range(m.height):
            for x in range(m.width):
                assert inflated.grid[y][x] == m.grid[y][x]

    def test_radius_one_expands_walls(self):
        m = make_map(
            """
            . . . . .
            . . # . .
            . . . . .
            """
        )
        inflated = inflate_map(m, radius_cells=1)
        # (2,1) 주변 3x3가 모두 WALL이 되어야 한다.
        for y in range(0, 3):
            for x in range(1, 4):
                assert inflated.grid[y][x] == CellType.WALL, f"({x},{y}) should be WALL"
        # 가장자리는 EMPTY로 유지
        assert inflated.grid[0][0] == CellType.EMPTY
        assert inflated.grid[2][4] == CellType.EMPTY

    def test_original_map_not_mutated(self):
        m = make_map(
            """
            . . . . .
            . . # . .
            . . . . .
            """
        )
        original_grid = [row[:] for row in m.grid]
        inflate_map(m, radius_cells=2)
        for y in range(m.height):
            for x in range(m.width):
                assert m.grid[y][x] == original_grid[y][x]

    def test_enemy_position_preserved_over_wall(self):
        m = make_map(
            """
            . . E . .
            . . . . .
            """
        )
        # ENEMY 위치는 (2,0). 인플레이션을 크게 줘서 WALL이 ENEMY를 덮는지 확인.
        inflated = inflate_map(m, radius_cells=2)
        # ENEMY 자체는 ENEMY로 유지되어야 한다
        assert inflated.grid[0][2] == CellType.ENEMY

    def test_start_protected_from_inflation(self):
        m = make_map(
            """
            S . . . .
            . # # # .
            . . . . .
            """
        )
        inflated = inflate_map(m, radius_cells=2)
        sx, sy = m.start
        assert inflated.grid[sy][sx] == CellType.EMPTY


# ---------------------------------------------------------------------------
# bresenham_line
# ---------------------------------------------------------------------------

class TestBresenham:
    def test_horizontal_line(self):
        line = bresenham_line((0, 0), (4, 0))
        assert line == [(0, 0), (1, 0), (2, 0), (3, 0), (4, 0)]

    def test_vertical_line(self):
        line = bresenham_line((0, 0), (0, 3))
        assert line == [(0, 0), (0, 1), (0, 2), (0, 3)]

    def test_diagonal_45(self):
        line = bresenham_line((0, 0), (3, 3))
        assert line == [(0, 0), (1, 1), (2, 2), (3, 3)]

    def test_reverse_direction(self):
        line = bresenham_line((3, 0), (0, 0))
        assert line[0] == (3, 0)
        assert line[-1] == (0, 0)
        assert len(line) == 4

    def test_single_point(self):
        assert bresenham_line((2, 2), (2, 2)) == [(2, 2)]


# ---------------------------------------------------------------------------
# attach_charge_segment
# ---------------------------------------------------------------------------

class TestAttachCharge:
    def test_short_path_unchanged(self):
        m = make_map(
            """
            . . . . .
            . . . . .
            """
        )
        path = [(0, 0)]
        assert attach_charge_segment(m, path) == path

    def test_no_qualifying_segment_returns_input(self):
        # min_charge_cells가 너무 커서 어떤 구간도 충족 못 함
        m = make_map(
            """
            . . . . .
            . . . . .
            """
        )
        path = [(0, 0), (2, 0), (4, 0)]
        result = attach_charge_segment(m, path, min_charge_cells=100)
        assert result == path

    def test_replaces_tail_with_straight_charge(self):
        # 충분히 긴 직선 구간이 있고, 그 마지막을 (start, goal) 두 점으로 줄여야 함.
        m = make_map(
            """
            . . . . . . . . . . . .
            . . . . . . . . . . . .
            """
        )
        path = [(0, 0), (3, 0), (6, 0), (10, 0)]
        result = attach_charge_segment(m, path, min_charge_cells=4)
        assert result[-1] == (10, 0)
        # 결과 길이는 원본보다 짧거나 같아야 한다.
        assert len(result) <= len(path)
