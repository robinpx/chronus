#!/usr/bin/env python3
# coding=utf-8
import time
from Arm_Lib import Arm_Device

# Get DOFBOT object
Arm = Arm_Device()
time.sleep(0.1)

print("Initialize Arm object")

# returns a list of angles for execution given file
def get_angle_list(file):
    with open(file, "r") as f:
        angles = f.read()
        angles = angles.split("\n")
        for i in range(len(angles)):
            line = angles[i].split(",")
            angles[i] = [float(n) for n in line]
        return angles
    return []


#  Writes in servo angles 1-5 or 1-6 with an option to set grip angle (servo 6) by taking in
# list of angle positions, a grip angle, and ms to run
def run_positions_set_grip(pos, grip_angle, ms):
    if len(pos) == 5 or len(pos) == 6:
        Arm.Arm_serial_servo_write6(
            pos[0], pos[1], pos[2], pos[3], pos[4], grip_angle, ms
        )
    else:
        raise IndexError("Angle position must be of length 5 or 6")


# Writes in servo angles into arm by taking in
# list of angle positions and ms to run
def run_positions(pos, ms):
    if len(pos) == 6:
        Arm.Arm_serial_servo_write6(
            pos[0], pos[1], pos[2], pos[3], pos[4], pos[5], 1000
        )
    else:
        raise IndexError("Angle position must be of length 6")


####### MAIN FUNCTION SEQUENCE ######
# Grips handle
def grip_handle():
    grip_angles = get_angle_list("./arm_commands/grip-circle-handle.txt")
    for pos in grip_angles:
        print(pos)
        # run_positions_set_grip(pos, 20, 1000)
        run_positions(pos, 1000)
        time.sleep(2)


# Draws circle
# takes in an integer for the amount of times a circle will be drawn
def draw_circle(times):
    if isinstance(times, int):
        circle_angles = get_angle_list("./arm_commands/draw-circle.txt")
        initial = circle_angles[0]
        print(initial)
        run_positions_set_grip(initial, 140, 2000)
        time.sleep(3)

        circle = circle_angles[1::]
        j = 0
        count = 0
        while j < len(circle):
            pos = circle[j]
            print(pos)
            run_positions_set_grip(pos, 140, 200)
            time.sleep(100 / 1000)
            j += 1
            if j >= len(circle) and count < times - 1:
                j = 0
                count += 1
    else:
        raise TypeError("Must pass an int")


def release_grip():
    release_angles = get_angle_list("./arm_commands/release-circle-handle.txt")
    for pos in release_angles:
        print(pos)
        run_positions(pos, 1000)
        time.sleep(2)


def main():
    print("Arm begins drawing circle action")
    grip_handle()
    draw_circle(2)
    release_grip()
    print("Arm finish drawing circle action")


try:
    main()
    del Arm  # Release DOFBOT object
    print("Arm object released")
except KeyboardInterrupt:
    print("Program closed!")
    pass
