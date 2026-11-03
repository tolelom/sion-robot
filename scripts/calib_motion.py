#!/usr/bin/env python3
"""모터 캘리브레이션: 일정 속도로 직진·회전시킨 뒤 이동 거리/각도를 손으로 재서
config.SCALE_V_MPS·SCALE_OMEGA_RAD를 정한다. 실제 JETANK에서만 동작한다.

사용: python -m scripts.calib_motion   (필요한 테스트의 주석을 풀고 실행)
"""
import time

from jetbot import Robot

robot = Robot()

def drive_test(speed, duration):
    print(f"\n=== DRIVE TEST: speed={speed}, duration={duration}s ===")
    robot.set_motors(speed, speed)
    time.sleep(duration)
    robot.set_motors(0, 0)
    print(">>> Measure the forward distance (cm) and write it down.\n")
    time.sleep(2)


def turn_test(turn_speed, duration):
    print(f"\n=== TURN TEST: turn_speed={turn_speed}, duration={duration}s ===")
    # 좌-우 반대방향으로 회전
    robot.set_motors(-turn_speed, turn_speed)
    time.sleep(duration)
    robot.set_motors(0, 0)
    print(">>> Measure the rotation angle (degrees) and write it down.\n")
    time.sleep(2)


if __name__ == "__main__":
    print("=== Calibration Start ===")
    time.sleep(2)

    # 직진 테스트
    #drive_test(0.2, 2.0)
    #drive_test(0.4, 2.0)
    #drive_test(0.6, 2.0)

    # 회전 테스트
    #turn_test(0.2, 2.0)
    #turn_test(0.4, 2.0)
    #turn_test(0.6, 2.0)

    robot.set_motors(0, 0)
    print("=== Calibration Done ===")
