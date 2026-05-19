"""ModeManager 단위 테스트.

대상: apply_server_command (모드 변경/목표 파싱), tick (auto 모드 랜덤 목표 생성,
manual 모드 pending → goal 승격), _parse_goal 에러 케이스.
plan_path_for_goal은 pytest-mock으로 모킹해 결정적 결과를 보장.
"""
import time
from unittest.mock import MagicMock

import pytest

import mode_manager as mode_manager_module
from mode_manager import ModeManager
from state import AGVState
from tests.conftest import make_map


@pytest.fixture
def m():
    return make_map(
        """
        . . . . .
        . . . . .
        . . . . .
        . . . . .
        . . . . .
        """
    )


@pytest.fixture
def state():
    return AGVState()


@pytest.fixture
def mm(state, m):
    return ModeManager(state, m)


# ---------------------------------------------------------------------------
# apply_server_command
# ---------------------------------------------------------------------------

class TestApplyServerCommand:
    def test_mode_change_to_manual(self, mm, state):
        mm.apply_server_command({"mode": "manual"}, is_moving=False)
        assert state.get_mode() == "manual"

    def test_mode_change_ignored_for_invalid_value(self, mm, state):
        # "turbo" 같은 알 수 없는 값은 무시되어야 한다 — set_mode가 그대로 호출되면 assert 실패.
        original = state.get_mode()
        mm.apply_server_command({"mode": "turbo"}, is_moving=False)
        assert state.get_mode() == original

    def test_cell_goal_when_not_moving_sets_goal_directly(self, mm, state):
        mm.apply_server_command(
            {"goal": {"type": "cell", "x": 3, "y": 4}}, is_moving=False
        )
        assert state.get_goal_cell() == (3, 4)
        assert state.get_pending_goal_cell() is None

    def test_cell_goal_when_moving_goes_to_pending(self, mm, state):
        mm.apply_server_command(
            {"goal": {"type": "cell", "x": 2, "y": 1}}, is_moving=True
        )
        assert state.get_goal_cell() is None
        assert state.get_pending_goal_cell() == (2, 1)

    def test_world_goal_is_converted_to_cell(self, mm, state, m):
        # res=0.1 기준 (0.25, 0.35) → (2, 3)
        mm.apply_server_command(
            {"goal": {"type": "world", "x": 0.25, "y": 0.35}}, is_moving=False
        )
        assert state.get_goal_cell() == (2, 3)

    def test_is_enemy_goal_flag_is_persisted(self, mm, state):
        mm.apply_server_command(
            {"goal": {"type": "cell", "x": 1, "y": 1}, "is_enemy_goal": True},
            is_moving=False,
        )
        assert state.is_enemy_goal() is True

    def test_missing_goal_records_error(self, mm, state):
        mm.apply_server_command({}, is_moving=False)
        assert state.get_last_error() == "no goal in command"
        assert state.get_goal_cell() is None

    def test_goal_out_of_range_records_error(self, mm, state):
        mm.apply_server_command(
            {"goal": {"type": "cell", "x": 99, "y": 0}}, is_moving=False
        )
        assert "out of map range" in state.get_last_error()
        assert state.get_goal_cell() is None

    def test_goal_unknown_type_records_error(self, mm, state):
        mm.apply_server_command(
            {"goal": {"type": "polar", "x": 1, "y": 1}}, is_moving=False
        )
        assert "unknown goal.type" in state.get_last_error()

    def test_goal_missing_xy_records_error(self, mm, state):
        mm.apply_server_command(
            {"goal": {"type": "cell", "x": 1}}, is_moving=False
        )
        assert "goal.x / goal.y missing" in state.get_last_error()

    def test_goal_must_be_dict(self, mm, state):
        mm.apply_server_command({"goal": "anywhere"}, is_moving=False)
        assert "goal must be a dict" in state.get_last_error()


# ---------------------------------------------------------------------------
# tick — manual 모드
# ---------------------------------------------------------------------------

class TestTickManual:
    def test_promotes_pending_goal_when_idle(self, mm, state):
        state.set_mode("manual")
        state.set_pending_goal_cell((2, 2))
        mm.tick(current_cell=(0, 0), is_moving=False)
        assert state.get_goal_cell() == (2, 2)
        assert state.get_pending_goal_cell() is None

    def test_does_not_promote_when_moving(self, mm, state):
        state.set_mode("manual")
        state.set_pending_goal_cell((2, 2))
        mm.tick(current_cell=(0, 0), is_moving=True)
        assert state.get_goal_cell() is None
        assert state.get_pending_goal_cell() == (2, 2)

    def test_does_not_promote_when_active_goal_exists(self, mm, state):
        state.set_mode("manual")
        state.set_goal_cell((1, 1))
        state.set_pending_goal_cell((9, 9))
        mm.tick(current_cell=(0, 0), is_moving=False)
        # 활성 goal이 있으면 pending이 그대로 남아야 함
        assert state.get_goal_cell() == (1, 1)
        assert state.get_pending_goal_cell() == (9, 9)


# ---------------------------------------------------------------------------
# tick — auto 모드
# ---------------------------------------------------------------------------

class TestTickAuto:
    def test_clears_pending_goal_in_auto_mode(self, mm, state):
        state.set_mode("auto")
        state.set_pending_goal_cell((3, 3))
        mm.tick(current_cell=(0, 0), is_moving=False)
        # auto에서는 pending 개념이 없음
        assert state.get_pending_goal_cell() is None

    def test_auto_picks_random_goal_when_interval_elapsed(self, mocker, mm, state):
        state.set_mode("auto")
        # 마지막 자동 시각을 충분히 과거로 → 인터벌 만료
        mm._last_auto_time = time.time() - 1000

        # pick_random_goal이 (2, 2)를 반환하도록 직접 패치
        mocker.patch.object(mm, "pick_random_goal", return_value=(2, 2))

        mm.tick(current_cell=(0, 0), is_moving=False)
        assert state.get_goal_cell() == (2, 2)
        assert state.is_enemy_goal() is False

    def test_auto_does_not_overwrite_existing_goal(self, mocker, mm, state):
        state.set_mode("auto")
        state.set_goal_cell((9, 9))
        mocker.patch.object(mm, "pick_random_goal", return_value=(2, 2))
        mm._last_auto_time = time.time() - 1000
        mm.tick(current_cell=(0, 0), is_moving=False)
        # 기존 goal 유지
        assert state.get_goal_cell() == (9, 9)

    def test_auto_skips_when_within_interval(self, mocker, mm, state):
        state.set_mode("auto")
        mm._last_auto_time = time.time()  # 방금 호출됨
        spy = mocker.patch.object(mm, "pick_random_goal", return_value=(2, 2))
        mm.tick(current_cell=(0, 0), is_moving=False)
        spy.assert_not_called()
        assert state.get_goal_cell() is None

    def test_auto_skips_when_moving(self, mocker, mm, state):
        state.set_mode("auto")
        mm._last_auto_time = time.time() - 1000
        spy = mocker.patch.object(mm, "pick_random_goal", return_value=(2, 2))
        mm.tick(current_cell=(0, 0), is_moving=True)
        spy.assert_not_called()


# ---------------------------------------------------------------------------
# pick_random_goal — plan_path_for_goal 모킹
# ---------------------------------------------------------------------------

class TestPickRandomGoal:
    def test_returns_first_planable_goal(self, mocker, mm):
        # plan_path_for_goal이 항상 경로를 찾음 → 첫 시도가 성공
        mocker.patch.object(
            mode_manager_module, "plan_path_for_goal",
            return_value=([(0, 0), (1, 1)], False),
        )
        goal = mm.pick_random_goal((0, 0))
        assert goal is not None
        gx, gy = goal
        assert 0 <= gx < mm._map.width
        assert 0 <= gy < mm._map.height

    def test_returns_none_when_no_path_in_any_sample(self, mocker, mm):
        mocker.patch.object(
            mode_manager_module, "plan_path_for_goal",
            return_value=(None, False),
        )
        assert mm.pick_random_goal((0, 0)) is None

    def test_skips_start_cell(self, mocker, mm):
        # plan_path_for_goal를 호출하지 않더라도, start_cell과 같은 셀은 건너뛰어야 한다.
        # 항상 동일 셀이 뽑히도록 random.randint를 패치.
        mocker.patch("mode_manager.random.randint", return_value=0)
        # plan_path_for_goal가 호출되면 안 됨 (전부 start와 동일)
        plan_spy = mocker.patch.object(
            mode_manager_module, "plan_path_for_goal",
            return_value=([(0, 0)], False),
        )
        assert mm.pick_random_goal((0, 0)) is None
        plan_spy.assert_not_called()
