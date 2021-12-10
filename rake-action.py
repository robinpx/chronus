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

def grip_rake():
    release_angles = get_angle_list("./arm_commands/grip-rake-low.txt")
    for pos in release_angles:
        print(pos)
        run_positions(pos, 1000)
        time.sleep(2)


def release_rake():
    release_angles = get_angle_list("./arm_commands/release-rake.txt")
    for pos in release_angles:
        print(pos)
        run_positions(pos, 1000)
        time.sleep(2)

def position_rake():
    release_angles = get_angle_list("./arm_commands/position-rake.txt")
    for pos in release_angles:
        print(pos)
        run_positions(pos, 1500)
        time.sleep(2)


def draw_rake():
    release_angles = get_angle_list("./arm_commands/draw-rake.txt")
    for pos in release_angles:
        print(pos)
        run_positions(pos, 1200)
        time.sleep(1.2)


def draw_rake_rand():
    release_angles = get_angle_list("./arm_commands/draw-rake.txt")
    first_deg = random.randint(-15,15)
    for pos in release_angles:
        pos[0]=first_deg
        print(pos)
        run_positions(pos, 1200)
        time.sleep(1.2)
    
def default_position():
    pos=[0,90,90,90,90,160]
    run_positions(pos,1500)

def main():
    print("Arm begins rake action")
    start = time.perf_counter()
    now = time.perf_counter()
    grip_rake()
    position_rake()
    mins = (random.randint(1,4))
    print("Running for " + str(mins) + " mins")
    while now - start < 60 * (random.randint(1,4)):
        print("elapsed seconds: ", now - start)
        draw_rake_rand()
        now = time.perf_counter()
    release_rake()
    default_position()
    print("Arm finishes rake action")


try:
    main()
    del Arm  # Release DOFBOT object
    print("Arm object released")
except KeyboardInterrupt:
    print("Program closed!")
    pass