#!/usr/bin/env python3

import os, sys, math
from adafruit_servokit import ServoKit
from revolute_robot import RevoluteRobot

kit = ServoKit(channels=16)
program_running = True
sim_running = False

def init_servos(kit):
    for i in range(1, 6, 1):
        kit.servo[i].set_pulse_width_range(500, 2500)
    kit.servo[0].set_pulse_width_range(1800, 2300)

def build_robot():
    # Create robot object
    robot = RevoluteRobot()
    # Define its joints
    robot.add_joint_link(length=9.4,  min_theta= 0, max_theta=360, theta=90)
    robot.add_joint_link(length=10.5, min_theta= 0, max_theta=360, theta=180)
    robot.add_joint_link(length=10.2, min_theta= 0, max_theta=360, theta=90)
    robot.add_joint_link(length=13.0, min_theta= 0, max_theta=360, theta=0)
    return robot

def get_coordinates():
    global program_running
    global sim_running
    print("Enter desired robot end effector position:")
    val = input("Format: X,Y,Z")
    if val == 'q':
        program_running = False
        return False
    elif val == 'z':
        sim_running = True
        return False
    elif val == 'o':
        kit.servo[0].angle=0
        return False
    elif val == 'c':
        kit.servo[0].angle=180
        return False
    else:
        try:
            x, y, z = val.replace(' ', '').split('/')
            x = int(x)
            y = int(y)
            z = int(z)
            return x, y, z
        except:
            print("Error, Invalid coordinates received.")
            return False

if __name__=="__main__":
    init_servos()
    myRobot = build_robot()
    myRobot.set_target(0, -20, 15)
    t = 0
    while program_running:
        if myRobot.on_target() and not sim_running:
            coordinates = get_coordinates()
            if coordinates:
                x, y, z = coordinates
                myRobot.set_target(x, -y, z)
        elif sim_running:
            x1 = 1.1 * math.cos(0.12 * t) * 10 * math.cos(t)
            y1 = -20
            z1 = 15 + (1.1 * math.cos(0.2 * t) * 10 * math.sin(t))
            myRobot.set_target(x1, y1, z1)
            t += 0.025
            if t > 11:
                t = 0
                sim_running = False
        else:
            myRobot.move_to_target()
            joint = 5
            for theta in myRobot.theta['theta']:
                angle = math.degrees(theta)
                kit.servo(joint).angle = angle
                joint -= 1
