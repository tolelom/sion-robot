import json
from enum import IntEnum
from typing import List, Tuple, Optional


class CellType(IntEnum):
    EMPTY = 0
    WALL = 1
    ENEMY = 2


class MapData:
    def __init__(
        self,
        width: int,
        height: int,
        resolution: float,
        grid: List[List[CellType]],
        start: Optional[Tuple[int, int]],
        enemies: List[Tuple[int, int]],
    ) -> None:
        self.width = width
        self.height = height
        self.resolution = resolution
        self.grid = grid  # grid[y][x]
        self.start = start
        self.enemies = enemies


def load_map(path: str) -> MapData:
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)

    width = data["width"]
    height = data["height"]
    res = data["resolution"]
    rows = data["cells"]

    if len(rows) != height:
        raise ValueError(f"height mismatch: declared={height}, actual={len(rows)}")

    grid: List[List[CellType]] = []
    start: Optional[Tuple[int, int]] = None
    enemies: List[Tuple[int, int]] = []

    for y, row_str in enumerate(rows):
        if len(row_str) != width:
            raise ValueError(f"width mismatch on row {y}: declared={width}, actual={len(row_str)}")
        row: List[CellType] = []
        for x, ch in enumerate(row_str):
            if ch == "#":
                cell = CellType.WALL
            elif ch == "E":
                cell = CellType.ENEMY
                enemies.append((x, y))
            else:
                cell = CellType.EMPTY
                if ch == "S":
                    start = (x, y)
            row.append(cell)
        grid.append(row)

    return MapData(width, height, res, grid, start, enemies)


def cell_to_world(x_idx: int, y_idx: int, res: float) -> Tuple[float, float]:
    """셀 중심의 월드 좌표(m)."""
    x = (x_idx + 0.5) * res
    y = (y_idx + 0.5) * res
    return x, y


def world_to_cell(x: float, y: float, res: float) -> Tuple[int, int]:
    x_idx = int(x // res)
    y_idx = int(y // res)
    return x_idx, y_idx


if __name__ == "__main__":
    import sys

    filename = sys.argv[1] if len(sys.argv) > 1 else "map_jetank_60x60.json"
    m = load_map(filename)
    print("Map loaded:")
    print(" size:", m.width, "x", m.height)
    print(" resolution:", m.resolution)
    print(" start:", m.start)
    print(" enemies:", m.enemies)
