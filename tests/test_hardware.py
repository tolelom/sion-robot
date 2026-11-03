"""hardware.py 단위 테스트 — 실제 jetbot/SCSCtrl 대신 가짜 모듈을 sys.modules에 끼운다."""
import sys
import types

import pytest

import config
import hardware
from hardware import DummyRobot, JetankRobot, create_robot, mix_differential


class TestMixDifferential:
    def test_straight(self):
        assert mix_differential(0.5, 0.0) == (0.5, 0.5)

    def test_turn_in_place_ccw(self):
        left, right = mix_differential(0.0, 1.0, omega_gain=0.7)
        assert left == pytest.approx(-0.7)
        assert right == pytest.approx(0.7)

    def test_inputs_and_outputs_clamped(self):
        left, right = mix_differential(5.0, 5.0, omega_gain=0.7)
        assert (left, right) == (pytest.approx(0.3), 1.0)
        assert mix_differential(-3.0, 0.0) == (-1.0, -1.0)


@pytest.fixture
def fake_hw(monkeypatch):
    calls = {"motors": [], "servos": []}

    class FakeJetbotRobot:
        def set_motors(self, left, right):
            calls["motors"].append((left, right))

    class FakeTTLServo:
        @staticmethod
        def servoAngleCtrl(sid, angle, speed, time):
            calls["servos"].append((sid, angle, speed, time))

    monkeypatch.setitem(sys.modules, "jetbot", types.SimpleNamespace(Robot=FakeJetbotRobot))
    monkeypatch.setitem(sys.modules, "SCSCtrl", types.SimpleNamespace(TTLServo=FakeTTLServo))
    return calls


class TestJetankRobot:
    def test_init_moves_servos_to_initial_pose(self, fake_hw):
        JetankRobot()
        expected = [(sid, a, config.SERVO_SPEED, config.SERVO_TIME) for sid, a in config.SERVO_INIT_ANGLES.items()]
        assert fake_hw["servos"] == expected

    def test_velocity_stop_servo(self, fake_hw):
        r = JetankRobot()
        r.set_velocity(0.5, 0.0)
        r.stop()
        r.set_servo(2, 10, 1, 100)
        assert fake_hw["motors"] == [(0.5, 0.5), (0.0, 0.0)]
        assert fake_hw["servos"][-1] == (2, 10, 1, 100)


class TestCreateRobot:
    def test_dummy_flag(self):
        assert isinstance(create_robot(dummy=True), DummyRobot)

    def test_falls_back_when_hardware_missing(self, monkeypatch):
        monkeypatch.setitem(sys.modules, "jetbot", None)  # import 시 ImportError
        assert isinstance(create_robot(), DummyRobot)

    def test_uses_hardware_when_available(self, fake_hw):
        assert isinstance(create_robot(), JetankRobot)


def test_dummy_robot_methods_are_noops():
    r = DummyRobot()
    r.set_velocity(0.1, 0.2)
    r.stop()
    r.set_servo(1, 0)
    assert isinstance(r, hardware.RobotBase)
