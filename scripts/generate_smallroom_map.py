"""map_smallroom_60x60.json을 다시 만든다.

사용: python -m scripts.generate_smallroom_map
"""
import json
import os

from config import BASE_DIR

WIDTH, HEIGHT = 60, 60
RESOLUTION_M = 0.05  # 5cm
OUTPUT_FILE = os.path.join(BASE_DIR, "map_smallroom_60x60.json")


def build_cells():
    cells = [["."] * WIDTH for _ in range(HEIGHT)]

    def set_char(x, y, ch):
        # y: 0이 맵의 아래쪽, HEIGHT-1이 위쪽
        cells[y][x] = ch

    # 시작점 S (약 (0.25m, 0.25m))
    set_char(4, 4, "S")

    # 적 E (약 (0.85m, 0.85m))
    set_char(16, 16, "E")

    # 세로벽: x = 10, y = 3..18
    for y in range(3, 19):
        set_char(10, y, "#")

    # 가로벽: y = 12, x = 0..8
    for x in range(0, 9):
        set_char(x, 12, "#")

    # 작은 블럭: y = 8..9, x = 14..18
    for y in range(8, 10):
        for x in range(14, 19):
            set_char(x, y, "#")

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
