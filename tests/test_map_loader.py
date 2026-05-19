"""map_loader 단위 테스트.

대상: load_map (JSON → MapData), cell_to_world, world_to_cell 변환, 좌표 라운드트립.
"""
import json
from pathlib import Path

import pytest

from map_loader import CellType, cell_to_world, load_map, world_to_cell


def _write_map(tmp_path: Path, payload: dict) -> Path:
    f = tmp_path / "map.json"
    f.write_text(json.dumps(payload), encoding="utf-8")
    return f


# ---------------------------------------------------------------------------
# load_map
# ---------------------------------------------------------------------------

class TestLoadMap:
    def test_loads_minimal_map(self, tmp_path: Path):
        path = _write_map(
            tmp_path,
            {
                "width": 3,
                "height": 2,
                "resolution": 0.1,
                "cells": [
                    "...",
                    "...",
                ],
            },
        )
        m = load_map(str(path))
        assert m.width == 3
        assert m.height == 2
        assert m.resolution == 0.1
        assert m.start is None
        assert m.enemies == []
        for row in m.grid:
            for cell in row:
                assert cell == CellType.EMPTY

    def test_parses_wall_enemy_start_chars(self, tmp_path: Path):
        path = _write_map(
            tmp_path,
            {
                "width": 5,
                "height": 3,
                "resolution": 0.05,
                "cells": [
                    "S.#.E",
                    ".....",
                    "##...",
                ],
            },
        )
        m = load_map(str(path))
        assert m.start == (0, 0)
        assert (4, 0) in m.enemies
        assert m.grid[0][2] == CellType.WALL
        assert m.grid[0][4] == CellType.ENEMY
        assert m.grid[2][0] == CellType.WALL
        assert m.grid[2][1] == CellType.WALL

    def test_width_mismatch_raises(self, tmp_path: Path):
        path = _write_map(
            tmp_path,
            {
                "width": 5,
                "height": 1,
                "resolution": 0.1,
                "cells": ["..."],  # 3글자뿐
            },
        )
        with pytest.raises(ValueError, match="width mismatch"):
            load_map(str(path))

    def test_height_mismatch_raises(self, tmp_path: Path):
        path = _write_map(
            tmp_path,
            {
                "width": 3,
                "height": 3,
                "resolution": 0.1,
                "cells": ["..."],  # 1행뿐
            },
        )
        with pytest.raises(ValueError, match="height mismatch"):
            load_map(str(path))


# ---------------------------------------------------------------------------
# 좌표 변환
# ---------------------------------------------------------------------------

class TestCoordinateConversion:
    def test_cell_to_world_center_of_cell(self):
        # 셀 (0,0)의 중심은 (0.5*res, 0.5*res)
        x, y = cell_to_world(0, 0, 0.1)
        assert x == pytest.approx(0.05)
        assert y == pytest.approx(0.05)

    def test_cell_to_world_arbitrary(self):
        x, y = cell_to_world(3, 4, 0.2)
        assert x == pytest.approx((3 + 0.5) * 0.2)
        assert y == pytest.approx((4 + 0.5) * 0.2)

    def test_world_to_cell_inside_cell(self):
        # (0.07, 0.13)은 res=0.1일 때 셀 (0, 1)에 속함
        assert world_to_cell(0.07, 0.13, 0.1) == (0, 1)

    def test_world_to_cell_at_boundary(self):
        # 정확히 res 배수면 다음 셀로 들어감
        assert world_to_cell(0.1, 0.0, 0.1) == (1, 0)
        assert world_to_cell(0.2, 0.2, 0.1) == (2, 2)

    def test_round_trip_cell_to_world_to_cell(self):
        res = 0.05
        for cx in range(0, 20, 3):
            for cy in range(0, 20, 3):
                wx, wy = cell_to_world(cx, cy, res)
                back = world_to_cell(wx, wy, res)
                assert back == (cx, cy)

    def test_negative_world_coords_round_to_negative_cell(self):
        # 음수 좌표는 floor div 동작 검증
        assert world_to_cell(-0.05, 0.0, 0.1) == (-1, 0)
