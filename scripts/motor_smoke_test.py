"""JETANK 모터 동작 확인: 1초 전진 후 정지. 실제 하드웨어에서만 동작한다.

사용: python -m scripts.motor_smoke_test
"""
import time

from jetbot import Robot


def main() -> None:
    print("Robot init...")
    r = Robot()
    print("Robot init OK, moving...")
    try:
        r.set_motors(0.3, 0.3)
        time.sleep(1.0)
    finally:
        r.stop()
    print("Done")


if __name__ == "__main__":
    main()
