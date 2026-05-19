"""AGVState 단위 테스트.

대상: 게터/세터, snapshot, promote_pending_goal, consume_goal,
멀티스레드 안전성.
"""
import threading
import time

import pytest

from state import AGVState


class TestStateDefaults:
    def test_initial_mode_is_auto(self):
        s = AGVState()
        assert s.get_mode() == "auto"

    def test_initial_goals_are_none(self):
        s = AGVState()
        assert s.get_goal_cell() is None
        assert s.get_pending_goal_cell() is None
        assert s.is_enemy_goal() is False

    def test_initial_pose_is_none(self):
        s = AGVState()
        assert s.get_pose_world() is None
        assert s.get_pose_cell() is None

    def test_initial_error_is_empty(self):
        s = AGVState()
        assert s.get_last_error() == ""


class TestSetters:
    def test_set_mode_accepts_auto_and_manual(self):
        s = AGVState()
        s.set_mode("manual")
        assert s.get_mode() == "manual"
        s.set_mode("auto")
        assert s.get_mode() == "auto"

    def test_set_mode_rejects_invalid(self):
        s = AGVState()
        with pytest.raises(AssertionError):
            s.set_mode("turbo")

    def test_set_goal_cell_round_trip(self):
        s = AGVState()
        s.set_goal_cell((3, 5))
        assert s.get_goal_cell() == (3, 5)
        s.set_goal_cell(None)
        assert s.get_goal_cell() is None

    def test_set_pose_updates_world_and_cell(self):
        s = AGVState()
        s.set_pose((1.0, 2.0, 0.5), (10, 20))
        assert s.get_pose_world() == (1.0, 2.0, 0.5)
        assert s.get_pose_cell() == (10, 20)

    def test_set_plan_result_failure_stores_error(self):
        s = AGVState()
        s.set_plan_result(False, "no path")
        assert s.get_last_error() == "no path"

    def test_set_plan_result_success_default_error_empty(self):
        s = AGVState()
        s.set_plan_result(True)
        assert s.get_last_error() == ""


class TestSnapshot:
    def test_snapshot_returns_copy_not_reference(self):
        s = AGVState()
        s.set_goal_cell((1, 1))
        snap = s.get_snapshot()
        # snapshot은 dict 복사본 — 외부 수정이 원본에 영향 없어야 함
        snap["goal_cell"] = (9, 9)
        assert s.get_goal_cell() == (1, 1)

    def test_snapshot_contains_all_keys(self):
        s = AGVState()
        snap = s.get_snapshot()
        expected_keys = {
            "mode", "goal_cell", "pending_goal_cell", "is_enemy_goal",
            "last_cmd_ts", "last_plan_ok", "last_error",
            "pose_world", "pose_cell",
        }
        assert set(snap.keys()) == expected_keys


class TestGoalLifecycle:
    def test_promote_pending_goal_moves_to_active(self):
        s = AGVState()
        s.set_pending_goal_cell((4, 4))
        promoted = s.promote_pending_goal()
        assert promoted == (4, 4)
        assert s.get_goal_cell() == (4, 4)
        assert s.get_pending_goal_cell() is None

    def test_promote_pending_goal_when_none_returns_none(self):
        s = AGVState()
        assert s.promote_pending_goal() is None
        assert s.get_goal_cell() is None

    def test_consume_goal_returns_and_clears(self):
        s = AGVState()
        s.set_goal_cell((7, 8))
        consumed = s.consume_goal()
        assert consumed == (7, 8)
        assert s.get_goal_cell() is None

    def test_consume_goal_when_none_returns_none(self):
        s = AGVState()
        assert s.consume_goal() is None


@pytest.mark.slow
class TestThreadSafety:
    def test_concurrent_set_get_does_not_corrupt(self):
        """100개 스레드가 각자 set/get을 반복해도 손상 없이 끝까지 도는지 확인."""
        s = AGVState()
        errors = []

        def worker(worker_id: int):
            try:
                for i in range(200):
                    s.set_goal_cell((worker_id, i))
                    _ = s.get_goal_cell()
                    s.set_mode("auto" if i % 2 == 0 else "manual")
                    _ = s.get_snapshot()
            except Exception as exc:  # pragma: no cover - 손상되면 캡처
                errors.append(exc)

        threads = [threading.Thread(target=worker, args=(i,)) for i in range(20)]
        for t in threads:
            t.start()
        for t in threads:
            t.join(timeout=10)
        assert errors == []
        # 끝나고도 mode가 유효 값이어야 함
        assert s.get_mode() in ("auto", "manual")

    def test_concurrent_promote_pending_goal_unique_owner(self):
        """여러 스레드가 동시에 promote_pending_goal을 호출해도
        같은 pending이 두 번 발견되지 않는다."""
        s = AGVState()
        s.set_pending_goal_cell((5, 5))

        winners = []
        lock = threading.Lock()

        def attempt():
            got = s.promote_pending_goal()
            if got is not None:
                with lock:
                    winners.append(got)

        threads = [threading.Thread(target=attempt) for _ in range(50)]
        for t in threads:
            t.start()
        for t in threads:
            t.join(timeout=2)

        # 정확히 한 스레드만 (5,5)를 받아야 한다
        assert winners == [(5, 5)]
