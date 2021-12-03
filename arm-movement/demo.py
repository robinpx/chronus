#!/usr/bin/env python3
# coding=utf-8
import time
from Arm_Lib import Arm_Device

# Get DOFBOT object
Arm = Arm_Device()
time.sleep(0.1)

# Control the movement of six servos at the same time, gradually changing the angle.
def ctrl_all_servo(angle, s_time=500):
    Arm.Arm_serial_servo_write6(angle, 180 - angle, angle, angle, angle, angle, s_time)
    time.sleep(s_time / 1000)


def main():
    file = open("demo.txt", "r")
    angles = file.read()
    angles = angles.split("\n")
    for i in range(len(angles)):
        line = angles[i].split(", ")
        # print(line)
        angles[i] = [float(n) for n in line]

    ##### GRIP HANDLE ######
    Arm.Arm_serial_servo_write6(30, 90, 90, 90, 90, 20, 1000)
    time.sleep(2)
    Arm.Arm_serial_servo_write6(-55, 90, 90, 90, 90, 20, 1000)
    time.sleep(2)
    Arm.Arm_serial_servo_write6(
        -55, 87.510888, 12.550138, 34.935173, 90.660412, 40, 1000
    )
    time.sleep(2)
    Arm.Arm_serial_servo_write6(
        -55, 87.510888, 12.550138, 34.935173, 90.660412, 140, 1000
    )
    time.sleep(2)
    Arm.Arm_serial_servo_write6(
        -55, 71.062461, 64.227874, -0.294137, 90.660439, 140, 1000
    )
    time.sleep(2)

    #     for i in range(4):
    #         state = angles[i]
    #         #print(state)
    #         Arm.Arm_serial_servo_write6(state[0], state[1], state[2], state[3], state[4], 130, 1000)
    #         time.sleep(2)

    #     Arm.Arm_serial_servo_write6(state[0], state[1], state[2], state[3], state[4], 130, 2000)
    #     time.sleep(2)

    state = angles[3]
    Arm.Arm_serial_servo_write6(
        state[0], state[1], state[2], state[3], state[4], 140, 2000
    )
    time.sleep(3)

    circle = angles[4::]
    j = 0
    count = 0
    while j < len(circle):
        state = circle[j]
        Arm.Arm_serial_servo_write6(
            state[0], state[1], state[2], state[3], state[4], 140, 200
        )
        time.sleep(100 / 1000)
        j += 1
        if j >= len(circle) and count < 2:
            j = 0
            count += 1

    ##### RELEASE GRIP #####
    Arm.Arm_serial_servo_write6(45, 90, 90, 90, 90, 140, 1000)
    time.sleep(2)
    Arm.Arm_serial_servo_write6(-55, 90, 90, 90, 90, 140, 1000)
    time.sleep(2)
    Arm.Arm_serial_servo_write6(
        -55, 71.062461, 64.227874, -0.294137, 90.660439, 140, 1000
    )
    time.sleep(2)
    Arm.Arm_serial_servo_write6(
        -55, 87.510888, 12.550138, 34.935173, 90.660412, 140, 2000
    )
    time.sleep(2)
    Arm.Arm_serial_servo_write6(
        -55, 87.510888, 12.550138, 34.935173, 90.660412, 40, 2000
    )
    time.sleep(2)

    Arm.Arm_serial_servo_write6(-30, 90, 90, 90, 90, 20, 1000)
    time.sleep(2)
    Arm.Arm_serial_servo_write6(
        -30, 87.510888, 12.550138, 34.935173, 90.660412, 40, 1000
    )
    time.sleep(2)
    #### RAKE HANDLE ####
    Arm.Arm_serial_servo_write6(
        -30, 90.250627, -34.562770, 79.240099, 87.126073, 120, 1000
    )
    time.sleep(4)

    Arm.Arm_serial_servo_write6(
        -30, 90.250627, -34.562770, 79.240099, 87.126073, 150, 1000
    )
    time.sleep(2)


try:
    main()
except KeyboardInterrupt:
    print(" Program closed! ")
    pass

del Arm  # Release DOFBOT object
