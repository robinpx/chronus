#!/usr/bin/env python3
#coding=utf-8
import time
from Arm_Lib import Arm_Device

# Get DOFBOT object
Arm = Arm_Device()
time.sleep(.1)

# Control the movement of six servos at the same time, gradually changing the angle.
def ctrl_all_servo(angle, s_time = 500):
    Arm.Arm_serial_servo_write6(angle, 180-angle, angle, angle, angle, angle, s_time)
    time.sleep(s_time/1000)


def main():
    file = open("demo.txt","r")
    angles = file.read()
    angles = angles.split("\n")
    for i in range(len(angles)):
        line = angles[i].split(", ")
        # print(line)
        angles[i] = [float(n) for n in line]
        
    for i in range(4):
        state = angles[i]
        #print(state)
        Arm.Arm_serial_servo_write6(state[0], state[1], state[2], state[3], state[4], 100, 1000)
        time.sleep(2)
    
    Arm.Arm_serial_servo_write6(state[0], state[1], state[2], state[3], state[4], 180, 2000)
    time.sleep(2)
    
    state = angles[3]
    Arm.Arm_serial_servo_write6(state[0], state[1], state[2], state[3], state[4], 180, 2000)
    time.sleep(2)
    
    circle = angles[4::]
    j=0
    while j < len(circle):
        state = circle[j]
        Arm.Arm_serial_servo_write6(state[0], state[1], state[2], state[3], state[4], 175, 200)
        time.sleep(100/1000)
        j+=1
        if j >= len(circle):
            j=0

try :
    main()
except KeyboardInterrupt:
    print(" Program closed! ")
    pass



del Arm  # Release DOFBOT object



