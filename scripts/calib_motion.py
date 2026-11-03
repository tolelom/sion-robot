#!/usr/bin/env python3
"""모터 캘리브레이션: 일정 속도로 직진·회전시킨 뒤 이동 거리/각도를 손으로 재서
config.SCALE_V_MPS·SCALE_OMEGA_RAD를 정한다. 실제 JETANK에서만 동작한다.

사용: python -m scripts.calib_motion {drive|turn} 속도 [속도 ...] [--duration 초]
"""
import argparse
import time

from jetbot import Robot

MEASURE_PAUSE_SEC = 2.0

robot = Robot()


def drive_test(speed, duration):
    print(f"\n=== DRIVE TEST: speed={speed}, duration={duration}s ===")
    robot.set_motors(speed, speed)
    time.sleep(duration)
    robot.set_motors(0, 0)
    print(">>> Measure the forward distance (cm) and write it down.\n")
    time.sleep(MEASURE_PAUSE_SEC)


def turn_test(turn_speed, duration):
    print(f"\n=== TURN TEST: turn_speed={turn_speed}, duration={duration}s ===")
    robot.set_motors(-turn_speed, turn_speed)
    time.sleep(duration)
    robot.set_motors(0, 0)
    print(">>> Measure the rotation angle (degrees) and write it down.\n")
    time.sleep(MEASURE_PAUSE_SEC)


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description="JETANK 모터 캘리브레이션")
    parser.add_argument("kind", choices=("drive", "turn"))
    parser.add_argument("speeds", type=float, nargs="+")
    parser.add_argument("--duration", type=float, default=2.0)
    args = parser.parse_args(argv)
    run_test = drive_test if args.kind == "drive" else turn_test

    print("=== Calibration Start ===")
    time.sleep(MEASURE_PAUSE_SEC)
    try:
        for speed in args.speeds:
            run_test(speed, args.duration)
    finally:
        robot.set_motors(0, 0)
    print("=== Calibration Done ===")


if __name__ == "__main__":
    main()
