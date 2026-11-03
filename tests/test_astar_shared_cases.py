"""sion-api algorithms/astar.go와 같은 픽스처로 astar_normal을 검사한다.

tests/fixtures/pathfinding-cases.json은 sion-api contract/pathfinding-cases.json의 복사본이다.
이 구현은 막힌 모서리도 대각으로 지나므로 corner_cutting_expected가 있으면 그 값을 기대한다.
"""
import json
import math
import os

import pytest

from astar import astar_normal
from tests.conftest import make_map

CASES_PATH = os.path.join(os.path.dirname(__file__), "fixtures", "pathfinding-cases.json")

with open(CASES_PATH, encoding="utf-8") as f:
    CASES = json.load(f)["cases"]


def _cost(path):
    return sum(math.dist(a, b) for a, b in zip(path, path[1:]))


@pytest.mark.parametrize("case", CASES, ids=[c["name"] for c in CASES])
def test_matches_shared_case(case):
    expected = case.get("corner_cutting_expected", case["expected"])
    start, goal = tuple(case["start"]), tuple(case["goal"])

    path = astar_normal(make_map("\n".join(case["map"])), start, goal)

    if expected is None:
        assert path is None
        return
    assert path is not None
    assert path[0] == start and path[-1] == goal
    assert len(path) == expected["cells"]
    assert _cost(path) == pytest.approx(expected["cost"], abs=1e-6)
    if "path" in expected:
        assert path == [tuple(p) for p in expected["path"]]
