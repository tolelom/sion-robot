"""path_planner.plan_path_for_goal 단위 테스트.

inflate → astar(normal/enemy) → smooth → (enemy면 charge segment 부착) 파이프라인 검증.
"""
from path_planner import plan_path_for_goal
from tests.conftest import make_map


def test_normal_goal_returns_path_without_charge_segment():
    m = make_map(
        """
        . . . . .
        . . . . .
        . . . . .
        . . . . .
        . . . . .
        """
    )
    path, used_enemy = plan_path_for_goal(
        m, start_cell=(0, 0), goal_cell=(4, 4),
        is_enemy_goal=False,
        inflate_radius=0,
        min_charge_cells=2,
    )
    assert used_enemy is False
    assert path is not None
    assert path[0] == (0, 0)
    assert path[-1] == (4, 4)


def test_unreachable_goal_returns_none():
    m = make_map(
        """
        . # . . .
        . # . . .
        . # . . .
        . # . . .
        . # . . .
        """
    )
    path, used_enemy = plan_path_for_goal(
        m, start_cell=(0, 0), goal_cell=(4, 4),
        is_enemy_goal=False,
        inflate_radius=0,
        min_charge_cells=2,
    )
    assert path is None
    assert used_enemy is False


def test_enemy_goal_uses_enemy_mode_and_attaches_charge_tail():
    m = make_map(
        """
        . . . . . . . . . . .
        . . . . . . . . . . .
        . . . . . . . . . . E
        """
    )
    path, used_enemy = plan_path_for_goal(
        m, start_cell=(0, 0), goal_cell=(10, 2),
        is_enemy_goal=True,
        inflate_radius=0,
        min_charge_cells=4,
    )
    assert used_enemy is True
    assert path is not None
    assert path[-1] == (10, 2)


def test_inflate_radius_blocks_narrow_passage():
    # 폭 1 통로를 inflate=1로 막아서 경로가 사라져야 한다
    m = make_map(
        """
        . . . . .
        # # . # #
        . . . . .
        """
    )
    # inflate 없으면 통과
    path0, _ = plan_path_for_goal(
        m, start_cell=(0, 0), goal_cell=(0, 2),
        is_enemy_goal=False,
        inflate_radius=0,
        min_charge_cells=2,
    )
    assert path0 is not None

    # inflate 1이면 통로가 사라져 unreachable
    path1, _ = plan_path_for_goal(
        m, start_cell=(0, 0), goal_cell=(0, 2),
        is_enemy_goal=False,
        inflate_radius=1,
        min_charge_cells=2,
    )
    assert path1 is None
