"""로봇 하드웨어 추상화: RobotBase 인터페이스와 JETANK·더미 구현."""
import logging
from abc import ABC, abstractmethod
from typing import Tuple

from config import OMEGA_MIX_GAIN, SERVO_INIT_ANGLES, SERVO_SPEED, SERVO_TIME

logger = logging.getLogger(__name__)


def _clamp_unit(value: float) -> float:
    return float(max(-1.0, min(1.0, value)))


def mix_differential(v_norm: float, omega_norm: float, omega_gain: float = OMEGA_MIX_GAIN) -> Tuple[float, float]:
    """정규화된 (선속도, 각속도)를 좌·우 바퀴 출력 [-1, 1]로 바꾼다(차동 구동)."""
    v_norm = _clamp_unit(v_norm)
    omega_norm = _clamp_unit(omega_norm)
    left = _clamp_unit(v_norm - omega_gain * omega_norm)
    right = _clamp_unit(v_norm + omega_gain * omega_norm)
    return left, right


class RobotBase(ABC):
    """모든 값은 정규화 단위: v_norm, omega_norm ∈ [-1, 1]."""

    @abstractmethod
    def set_velocity(self, v_norm: float, omega_norm: float) -> None: ...

    @abstractmethod
    def stop(self) -> None: ...

    @abstractmethod
    def set_servo(self, servo_id: int, angle: int,
                  speed: int = SERVO_SPEED, time: int = SERVO_TIME) -> None: ...


class JetankRobot(RobotBase):
    def __init__(self):
        from jetbot import Robot
        from SCSCtrl import TTLServo
        self._robot = Robot()
        self._TTLServo = TTLServo
        for sid, angle in SERVO_INIT_ANGLES.items():
            TTLServo.servoAngleCtrl(sid, angle, SERVO_SPEED, SERVO_TIME)
        logger.info("JetankRobot ready")

    def set_velocity(self, v_norm: float, omega_norm: float) -> None:
        left, right = mix_differential(v_norm, omega_norm)
        self._robot.set_motors(left, right)

    def stop(self) -> None:
        self._robot.set_motors(0.0, 0.0)

    def set_servo(self, servo_id: int, angle: int,
                  speed: int = SERVO_SPEED, time: int = SERVO_TIME) -> None:
        self._TTLServo.servoAngleCtrl(servo_id, angle, speed, time)


class DummyRobot(RobotBase):
    def __init__(self):
        logger.warning("DummyRobot: no hardware, logging only")

    def set_velocity(self, v_norm: float, omega_norm: float) -> None:
        logger.debug("(dummy) set_velocity v=%.2f omega=%.2f", v_norm, omega_norm)

    def stop(self) -> None:
        logger.debug("(dummy) stop")

    def set_servo(self, servo_id: int, angle: int,
                  speed: int = SERVO_SPEED, time: int = SERVO_TIME) -> None:
        logger.debug("(dummy) servo %d -> %d (speed=%d, time=%d)",
                     servo_id, angle, speed, time)


def create_robot(dummy: bool = False) -> RobotBase:
    """JETANK 하드웨어를 연다. jetbot/SCSCtrl을 import할 수 없으면 DummyRobot으로 대체한다.

    import 이외의 초기화 실패(시리얼 포트 등)는 그대로 전파한다 — 실제 로봇에서
    하드웨어 고장을 더미로 조용히 덮지 않기 위해서다.
    """
    if dummy:
        return DummyRobot()
    try:
        return JetankRobot()
    except ImportError:
        logger.warning("Hardware not available, falling back to DummyRobot")
        return DummyRobot()
