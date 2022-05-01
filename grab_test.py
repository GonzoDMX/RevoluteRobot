#!/usr/bin/env python3

import os, sys, math
from adafruit_servokit import ServoKit
from revolute_robot import RevoluteRobot

kit = ServoKit(channels=16)

program_running = True

def init_servos():
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
    print("Enter desired robot end effector position:")
    val = input("Format: X,Y,Z")
    if val == 'q':
        program_running = False
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
    while program_running
        if myRobot.on_target():
            coordinates = get_coordinates()
            if coordinates:
                x, y, z = coordinates
                myRobot.set_target(x, -y, z)
        else:
            myRobot.move_to_target()
            joint = 5
            for theta in myRobot.theta['theta']:
                angle = math.degrees(theta)
                kit.servo(joint).angle = angle
                joint -= 1
