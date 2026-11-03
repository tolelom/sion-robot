"""맵 JSON을 위쪽 행부터 텍스트로 출력한다.

사용: python -m scripts.show_map [맵 파일]   (기본: map_jetank_60x60.json)
"""
import json
import os
import sys

from sion_robot.config import BASE_DIR

DEFAULT_MAP = os.path.join(BASE_DIR, "map_jetank_60x60.json")


def main(argv=None) -> None:
    argv = sys.argv[1:] if argv is None else argv
    filename = argv[0] if argv else DEFAULT_MAP

    with open(filename, "r", encoding="utf-8") as f:
        data = json.load(f)

    print("width:", data["width"], "height:", data["height"])
    print("resolution:", data["resolution"], "m per cell")

    print("\nMap (top row first):")
    for row in reversed(data["cells"]):
        print(row)


if __name__ == "__main__":
    main()
