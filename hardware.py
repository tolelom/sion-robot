import logging
from abc import ABC, abstractmethod

from config import (K_OMEGA, SERVO_SPEED, SERVO_TIME, SERVO_INIT_ANGLES)

logger = logging.getLogger(__name__)


class RobotBase(ABC):
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
        v_norm = max(-1.0, min(1.0, v_norm))
        omega_norm = max(-1.0, min(1.0, omega_norm))
        left = v_norm - K_OMEGA * omega_norm
        right = v_norm + K_OMEGA * omega_norm
        left = float(max(-1.0, min(1.0, left)))
        right = float(max(-1.0, min(1.0, right)))
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
    if dummy:
        return DummyRobot()
    try:
        return JetankRobot()
    except ImportError:
        logger.warning("Hardware not available, falling back to DummyRobot")
        return DummyRobot()
