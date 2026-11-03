"""map_smallroom_60x60.json을 다시 만든다.

사용: python -m scripts.generate_smallroom_map
"""
import json
import os

from sion_robot.config import BASE_DIR

WIDTH, HEIGHT = 60, 60
RESOLUTION_M = 0.05
OUTPUT_FILE = os.path.join(BASE_DIR, "map_smallroom_60x60.json")

# y=0이 맵의 아래쪽 행이다.
START_CELL = (4, 4)
ENEMY_CELL = (16, 16)
WALL_RECTS = [  # (x 범위, y 범위)
    (range(10, 11), range(3, 19)),
    (range(0, 9), range(12, 13)),
    (range(14, 19), range(8, 10)),
]


def build_cells():
    cells = [["."] * WIDTH for _ in range(HEIGHT)]

    def set_char(cell, ch):
        x, y = cell
        cells[y][x] = ch

    set_char(START_CELL, "S")
    set_char(ENEMY_CELL, "E")
    for xs, ys in WALL_RECTS:
        for y in ys:
            for x in xs:
                set_char((x, y), "#")

    return ["".join(row) for row in cells]


def main() -> None:
    mp = {
        "width": WIDTH,
        "height": HEIGHT,
        "resolution": RESOLUTION_M,
        "cells": build_cells(),
    }
    with open(OUTPUT_FILE, "w", encoding="utf-8") as f:
        json.dump(mp, f, ensure_ascii=False, indent=2)
    print(f"Saved {OUTPUT_FILE}")


if __name__ == "__main__":
    main()
