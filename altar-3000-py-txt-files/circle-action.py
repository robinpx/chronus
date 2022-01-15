#!/usr/bin/env python3
# coding=utf-8
import time
import random
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
            pos[0], pos[1], pos[2], pos[3], pos[4], pos[5], ms)
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
def start_circle():
    circle_angles = get_angle_list("./arm_commands/draw-circle-smooth.txt")
    initial = circle_angles[0]
    run_positions_set_grip(initial, 135, 1000)
    time.sleep(2)

def draw_circle():
    circle_angles = get_angle_list("./arm_commands/draw-circle-smooth.txt")
    circle = circle_angles[1::]
    j = 0
    count = 0
    while j < len(circle):
        pos = circle[j]
        print(pos)
        run_positions_set_grip(pos, 135, 200)
        time.sleep(100 / 1000)
        j += 1

def position_circle():
    release_angles = get_angle_list("./arm_commands/position-circle.txt")
    for pos in release_angles:
        print(pos)
        run_positions(pos, 1000)
        time.sleep(2)

# releases the handle
def release_grip():
    release_angles = get_angle_list("./arm_commands/release-circle-handle.txt")
    for pos in release_angles:
        print(pos)
        run_positions(pos, 1000)
        time.sleep(2)
    
def default_position():
    pos=[0,90,90,90,90,160]
    run_positions(pos,1500)

def main():
    print("Arm begins drawing circle action")
    start = time.perf_counter()
    now = time.perf_counter()
    grip_handle()
    position_circle()
    start_circle()
    while now - start < 30:
        print("elapsed seconds: ", now - start)
        draw_circle()
        now = time.perf_counter()
    release_grip()
    default_position()
    print("Arm finish drawing circle action")


try:
    main()
    del Arm  # Release DOFBOT object
    print("Arm object released")
except KeyboardInterrupt:
    print("Program closed!")
    pass