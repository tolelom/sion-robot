"""follow_path 단위 테스트.

대상:
- PoseEstimator (순수 함수, 적분 검증)
- follow_path_constant_speed (RobotBase 스파이로 호출 시퀀스 검증)

테스트 시간 단축을 위해 매우 짧은 거리/scale로 호출하고 큰 cancel_event도 활용.
"""
import math
import threading
from typing import List, Tuple

import pytest

from follow_path import PoseEstimator, follow_path_constant_speed
from hardware import RobotBase


# ---------------------------------------------------------------------------
# PoseEstimator
# ---------------------------------------------------------------------------

class TestPoseEstimator:
    def test_initial_pose(self):
        p = PoseEstimator(1.0, 2.0, theta0=0.5)
        assert p.get_pose() == (1.0, 2.0, 0.5)

    def test_forward_motion_along_x(self):
        p = PoseEstimator(0.0, 0.0, theta0=0.0)
        p.update(v_mps=1.0, omega_rad=0.0, dt=1.0)
        x, y, th = p.get_pose()
        assert x == pytest.approx(1.0)
        assert y == pytest.approx(0.0)
        assert th == pytest.approx(0.0)

    def test_forward_motion_along_y_when_facing_north(self):
        p = PoseEstimator(0.0, 0.0, theta0=math.pi / 2)
        p.update(v_mps=2.0, omega_rad=0.0, dt=0.5)
        x, y, _ = p.get_pose()
        assert x == pytest.approx(0.0, abs=1e-9)
        assert y == pytest.approx(1.0)

    def test_rotation_only_updates_theta(self):
        p = PoseEstimator(5.0, 5.0, theta0=0.0)
        p.update(v_mps=0.0, omega_rad=math.pi, dt=0.5)
        x, y, th = p.get_pose()
        assert (x, y) == (5.0, 5.0)
        assert th == pytest.approx(math.pi / 2)

    def test_theta_wraps_to_minus_pi_plus_pi(self):
        # 큰 양의 회전이 누적되어도 theta는 (-pi, pi] 안에 머물러야 한다.
        p = PoseEstimator(0.0, 0.0, theta0=0.0)
        for _ in range(10):
            p.update(v_mps=0.0, omega_rad=math.pi, dt=1.0)
        _, _, th = p.get_pose()
        assert -math.pi <= th <= math.pi


# ---------------------------------------------------------------------------
# 스파이 로봇
# ---------------------------------------------------------------------------

class SpyRobot(RobotBase):
    """모든 호출을 기록하는 fake RobotBase."""

    def __init__(self) -> None:
        self.velocity_calls: List[Tuple[float, float]] = []
        self.stop_calls: int = 0
        self.servo_calls: List[Tuple[int, int, int, int]] = []

    def set_velocity(self, v_norm: float, omega_norm: float) -> None:
        self.velocity_calls.append((v_norm, omega_norm))

    def stop(self) -> None:
        self.stop_calls += 1

    def set_servo(self, servo_id, angle, speed=1, time=180) -> None:
        self.servo_calls.append((servo_id, angle, speed, time))


# ---------------------------------------------------------------------------
# follow_path_constant_speed
# ---------------------------------------------------------------------------

class TestFollowPath:
    def test_invalid_step_sec_raises(self):
        with pytest.raises(ValueError, match="invalid params"):
            follow_path_constant_speed(
                SpyRobot(),
                [(0.0, 0.0), (0.1, 0.0)],
                is_enemy_goal=False,
                step_sec=0.0,
            )

    def test_invalid_scale_raises(self):
        with pytest.raises(ValueError, match="invalid params"):
            follow_path_constant_speed(
                SpyRobot(),
                [(0.0, 0.0), (0.1, 0.0)],
                is_enemy_goal=False,
                scale_v_mps=0.0,
            )

    def test_single_waypoint_returns_pose_without_moving(self):
        robot = SpyRobot()
        x, y, th = follow_path_constant_speed(
            robot,
            [(1.5, 2.5)],
            is_enemy_goal=False,
        )
        assert (x, y, th) == (1.5, 2.5, 0.0)
        assert robot.velocity_calls == []

    def test_short_path_drives_and_stops(self):
        robot = SpyRobot()
        cancel = threading.Event()
        # 즉시 취소해서 실제 sleep 없이 빠르게 빠져나오게 한다 — 명령 시퀀스는 유지됨
        cancel.set()

        x, y, th = follow_path_constant_speed(
            robot,
            [(0.0, 0.0), (0.05, 0.0)],
            is_enemy_goal=False,
            cancel_event=cancel,
            step_sec=0.001,
        )
        # cancel 즉시 set이라도 finally의 robot.stop()이 반드시 한 번은 호출된다
        assert robot.stop_calls >= 1
        # 반환 pose는 시작점 근처 (이동 거의 안 함)
        assert (x, y) == (0.0, 0.0)

    def test_pose_callback_is_invoked(self):
        robot = SpyRobot()
        callback_poses: List[Tuple[float, float, float]] = []

        cancel = threading.Event()
        cancel.set()

        follow_path_constant_speed(
            robot,
            [(0.0, 0.0), (0.05, 0.0)],
            is_enemy_goal=False,
            pose_update_cb=lambda x, y, th: callback_poses.append((x, y, th)),
            cancel_event=cancel,
            step_sec=0.001,
        )
        # 초기 콜백 한 번은 보장 (safe_cb 호출)
        assert len(callback_poses) >= 1
        assert callback_poses[0] == (0.0, 0.0, 0.0)

    def test_pose_callback_exception_is_isolated(self):
        robot = SpyRobot()
        cancel = threading.Event()
        cancel.set()

        def bad_cb(*_):
            raise RuntimeError("boom")

        # 콜백이 예외를 던져도 함수 자체는 정상 반환되어야 한다
        follow_path_constant_speed(
            robot,
            [(0.0, 0.0), (0.05, 0.0)],
            is_enemy_goal=False,
            pose_update_cb=bad_cb,
            cancel_event=cancel,
            step_sec=0.001,
        )
        assert robot.stop_calls >= 1

    def test_zero_length_segment_is_skipped(self):
        robot = SpyRobot()
        cancel = threading.Event()
        cancel.set()
        # 같은 점이 연속하면 seg_len=0 → continue
        follow_path_constant_speed(
            robot,
            [(0.0, 0.0), (0.0, 0.0), (0.05, 0.0)],
            is_enemy_goal=False,
            cancel_event=cancel,
            step_sec=0.001,
        )
        # stop이 최소 한 번
        assert robot.stop_calls >= 1

    @pytest.mark.slow
    def test_runs_without_cancel_issues_velocity_commands(self):
        """짧은 거리 + 작은 scale로 실제 한 사이클 수행. 적분 정확도는
        Windows time.sleep 해상도에 영향 받아 불안정하므로 명령 발행 흐름만 검증."""
        robot = SpyRobot()
        follow_path_constant_speed(
            robot,
            [(0.0, 0.0), (0.01, 0.0)],
            is_enemy_goal=False,
            v_cruise_norm=1.0,
            scale_v_mps=1.0,       # 0.01m / 1m/s = 0.01s
            scale_omega_rad=1.0,
            step_sec=0.005,
        )
        # 종료 후 모터 정지 보장
        assert robot.stop_calls >= 1
        # DRIVE 단계의 set_velocity가 최소 한 번 호출되어야 한다
        # (각 run_for_duration이 robot.set_velocity로 시작)
        assert len(robot.velocity_calls) >= 1
