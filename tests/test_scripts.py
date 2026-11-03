"""scripts/ 보조 스크립트 스모크 테스트 (하드웨어가 필요 없는 것만)."""
import json

import config
from scripts import generate_smallroom_map, show_map


def test_generator_matches_committed_map():
    with open(config.MAP_FILE, encoding="utf-8") as f:
        committed = json.load(f)
    assert committed["cells"] == generate_smallroom_map.build_cells()
    assert committed["resolution"] == generate_smallroom_map.RESOLUTION_M


def test_show_map_prints_rows_top_first(capsys):
    show_map.main([config.MAP_FILE])
    out = capsys.readouterr().out
    assert "width: 60 height: 60" in out
    with open(config.MAP_FILE, encoding="utf-8") as f:
        cells = json.load(f)["cells"]
    lines = out.splitlines()
    first_row_idx = lines.index(cells[-1])
    assert lines[first_row_idx:first_row_idx + 2] == [cells[-1], cells[-2]]
