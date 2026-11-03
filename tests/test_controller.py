"""controller.py 단위 테스트 — 하드웨어·카메라 없이 move_worker와 초기화 로직을 검증."""
import threading
from typing import List, Tuple

import pytest

import controller
from hardware import RobotBase
from state import AGVState
from tests.conftest import make_map


class FakeRobot(RobotBase):
    def __init__(self):
        self.velocities: List[Tuple[float, float]] = []
        self.stops = 0

    def set_velocity(self, v_norm, omega_norm):
        self.velocities.append((v_norm, omega_norm))

    def stop(self):
        self.stops += 1

    def set_servo(self, servo_id, angle, speed=1, time=180):
        pass


OPEN_MAP = """
    S . . . .
    . . . . .
    . . . . .
"""


@pytest.fixture
def open_map():
    return make_map(OPEN_MAP, resolution=0.1)


class TestInitState:
    def test_pose_at_start_auto_mode(self, open_map):
        state = controller.init_state(open_map)
        assert state.get_mode() == "auto"
        assert state.get_pose_cell() == (0, 0)
        assert state.get_pose_world() == pytest.approx((0.05, 0.05, 0.0))
        assert state.get_pending_goal_cell() is None

    def test_enemy_becomes_pending_manual_goal(self):
        m = make_map("""
            S . . .
            . . . E
        """)
        state = controller.init_state(m)
        assert state.get_mode() == "manual"
        assert state.get_pending_goal_cell() == (3, 1)
        assert state.is_enemy_goal() is True

    def test_missing_start_raises(self):
        with pytest.raises(RuntimeError):
            controller.init_state(make_map(". . ."))


class TestMoveWorker:
    def test_success_reports_pose(self, open_map, monkeypatch):
        captured = {}

        def fake_follow(robot, waypoints_world, pose_update_cb, **kw):
            captured["waypoints"] = waypoints_world
            captured["kw"] = kw
            pose_update_cb(0.45, 0.05, 0.0)
            return (0.45, 0.05, 0.0)

        monkeypatch.setattr(controller, "follow_path_constant_speed", fake_follow)
        state = AGVState()
        controller.move_worker(FakeRobot(), open_map, (0, 0), (4, 0), False, state)

        snap = state.get_snapshot()
        assert snap["last_plan_ok"] is True
        assert captured["waypoints"][0] == pytest.approx((0.05, 0.05))
        assert captured["waypoints"][-1] == pytest.approx((0.45, 0.05))
        assert state.get_pose_cell() == (4, 0)

    def test_no_path(self, monkeypatch):
        m = make_map("""
            S # .
            # # .
            . . .
        """)
        monkeypatch.setattr(controller, "INFLATE_RADIUS", 0)
        called = []
        monkeypatch.setattr(controller, "follow_path_constant_speed", lambda **kw: called.append(kw))
        state = AGVState()
        controller.move_worker(FakeRobot(), m, (0, 0), (2, 2), False, state)
        assert state.get_snapshot()["last_plan_ok"] is False
        assert state.get_last_error() == "No path found"
        assert called == []

    def test_plan_exception_recorded(self, open_map, monkeypatch):
        def boom(*a, **kw):
            raise RuntimeError("planner broke")

        monkeypatch.setattr(controller, "plan_path_for_goal", boom)
        state = AGVState()
        controller.move_worker(FakeRobot(), open_map, (0, 0), (4, 0), False, state)
        assert "plan exception" in state.get_last_error()

    def test_follow_exception_recorded(self, open_map, monkeypatch):
        def boom(**kw):
            raise RuntimeError("motor fault")

        monkeypatch.setattr(controller, "follow_path_constant_speed", boom)
        state = AGVState()
        controller.move_worker(FakeRobot(), open_map, (0, 0), (4, 0), False, state)
        assert "follow exception" in state.get_last_error()

    def test_cancelled_follow_stops_robot(self, open_map):
        cancel = threading.Event()
        cancel.set()
        robot = FakeRobot()
        controller.move_worker(robot, open_map, (0, 0), (4, 0), False, AGVState(), cancel)
        assert robot.stops >= 1


class TestShutdown:
    def test_stops_robot_without_thread(self):
        robot = FakeRobot()
        controller._shutdown(robot, None)
        assert robot.stops == 1

    def test_robot_stop_exception_swallowed(self):
        class Broken(FakeRobot):
            def stop(self):
                raise OSError("bus error")

        controller._shutdown(Broken(), None)


def test_camera_thread_without_cv2_returns(monkeypatch):
    monkeypatch.setattr(controller, "cv2", None)
    controller.camera_thread_main(threading.Event())  # 즉시 반환해야 한다


def test_safe_release_swallows_errors():
    class Cap:
        def release(self):
            raise OSError("x")

    assert controller._safe_release(Cap()) is None
    assert controller._safe_release(None) is None


class TestEndCell:
    """회귀: 계획 실패 시 로봇 위치를 목표 셀로 착각하지 않아야 한다."""

    def test_move_worker_returns_start_when_no_path(self, monkeypatch):
        m = make_map("""
            S # .
            # # .
            . . .
        """)
        monkeypatch.setattr(controller, "INFLATE_RADIUS", 0)
        end = controller.move_worker(FakeRobot(), m, (0, 0), (2, 2), False, AGVState())
        assert end == (0, 0)

    def test_move_worker_returns_goal_after_follow(self, open_map, monkeypatch):
        monkeypatch.setattr(controller, "follow_path_constant_speed", lambda **kw: None)
        end = controller.move_worker(FakeRobot(), open_map, (0, 0), (4, 0), False, AGVState())
        assert end == (4, 0)

    def test_move_job_exposes_end_cell(self, open_map, monkeypatch):
        monkeypatch.setattr(controller, "follow_path_constant_speed", lambda **kw: None)
        job = controller.MoveJob(FakeRobot(), open_map, (0, 0), (4, 0), False, AGVState(), threading.Event())
        assert job.end_cell == (0, 0)
        job.start()
        job.join(2.0)
        assert not job.is_alive()
        assert job.end_cell == (4, 0)
