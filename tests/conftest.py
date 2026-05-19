"""pytest 공통 fixture.

sion/ 디렉토리의 모듈들은 `from map_loader import ...` 처럼 상대 import를 쓰므로
sys.path에 sion/ 디렉토리를 추가해 테스트에서도 같은 import가 동작하도록 한다.
"""
import os
import sys
from typing import List

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from map_loader import CellType, MapData  # noqa: E402


def _parse_grid(grid_str: str) -> List[List[CellType]]:
    """공백/개행으로 정리된 문자열을 grid[y][x] 리스트로 변환.

    문자 매핑:
      '.', ' '  -> EMPTY
      '#'       -> WALL
      'E'       -> ENEMY
      'S'       -> EMPTY (시작 표시는 호출자가 별도로 알아냄)
    """
    rows = [line.strip() for line in grid_str.strip().splitlines() if line.strip()]
    width = len(rows[0])
    for y, row in enumerate(rows):
        if len(row) != width:
            raise ValueError(
                f"row {y} length mismatch: expected {width}, got {len(row)} ({row!r})"
            )

    grid: List[List[CellType]] = []
    for row in rows:
        line: List[CellType] = []
        for ch in row:
            if ch == "#":
                line.append(CellType.WALL)
            elif ch == "E":
                line.append(CellType.ENEMY)
            else:
                line.append(CellType.EMPTY)
        grid.append(line)
    return grid


def make_map(grid_str: str, resolution: float = 0.1) -> MapData:
    """문자열 그리드로 MapData를 만든다. 'S' 위치는 start로 기록.

    예:
        m = make_map('''
            . . . . .
            . # # # .
            . . . . .
        ''')
    """
    # 공백/개행 normalize
    compact = "\n".join(
        "".join(ch for ch in line if not ch.isspace())
        for line in grid_str.strip().splitlines()
        if line.strip()
    )
    grid = _parse_grid(compact)
    height = len(grid)
    width = len(grid[0])

    rows = [line for line in compact.splitlines() if line]
    start = None
    enemies: List = []
    for y, row in enumerate(rows):
        for x, ch in enumerate(row):
            if ch == "S":
                start = (x, y)
            elif ch == "E":
                enemies.append((x, y))

    return MapData(
        width=width,
        height=height,
        resolution=resolution,
        grid=grid,
        start=start,
        enemies=enemies,
    )


@pytest.fixture
def make_map_fixture():
    """팩토리 fixture — 테스트 안에서 make_map(...) 처럼 호출."""
    return make_map
