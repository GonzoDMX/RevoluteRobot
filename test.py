#!/usr/bin/env python3

import sys
import numpy as np
import math, random
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d import proj3d
from revolute_robot import RevoluteRobot

myRobot = RevoluteRobot()

def print_bot():
    print()
    print("Thetas: {}".format([math.degrees(i) for i in myRobot.theta['theta']]))
    print("Lengths: {}".format(myRobot.lengths))
    print("X: {}".format([round(i, 2) for i in myRobot.joints[0, :]]))
    print("Y: {}".format([round(i, 2) for i in myRobot.joints[1, :]]))
    print("Z: {}".format([round(i, 2) for i in myRobot.joints[2, :]]))

myRobot.add_joint_link(length=10,  min_theta= 0,   max_theta=360, theta=90)
myRobot.add_joint_link(length=10, min_theta= 0,   max_theta=360, theta=180)
myRobot.add_joint_link(length=10, min_theta= 0, max_theta=360, theta=90)
myRobot.add_joint_link(length=5.0, min_theta= 0, max_theta=360, theta=0)
#myRobot.update_posture()
#print_bot()

#sys.exit(0)

fig = plt.figure(figsize=(20, 20))
ax = fig.add_subplot(111, projection='3d')
fig.subplots_adjust(left=0.05, bottom=0.05, right=1, top=1)

#ax.plot([0, 3], [0, 6], [0, 10])
armPlot, = ax.plot(myRobot.joints[0, :], myRobot.joints[1, :], myRobot.joints[2, :], marker='o', c='g', lw=2)
targetPt, = ax.plot(0, 0, 0, marker="x", markersize=20, c="red")

ax.set_xlabel('$X$', fontsize=20)
ax.set_ylabel('$Y$', fontsize=20)
ax.set_zlabel('$Z$', fontsize=20)

ax.set_xlim(-30, 30)
ax.set_ylim(-30, 30)
ax.set_zlim(0, 30)

# Set labels for joints
joint_labels = []
for i in range(myRobot.joint_count):
    xr = myRobot.joints[0, i]
    yr =  myRobot.joints[1, i]
    zr = myRobot.joints[2, i]
    rot = math.degrees(myRobot.theta['theta'][i])
    t = ax.text2D(xr, yr,
        "Joint {}: X({}), Y({}), Z({}), Rot({})".format( i,
                                            round(xr, 2),
                                            round(yr, 2),
                                            round(zr, 2),
                                            round(rot, 2)),
                                            fontsize= 'xx-large')
    joint_labels.append(t)

# Set label for end effector
xe = myRobot.joints[0, -1]
ye =  myRobot.joints[1, -1]
ze = myRobot.joints[2, -1]
effector = ax.text2D(xe, ye, "End Effector: X({}), Y({}), Z({})".format(
                    round(xe, 2),round(ye, 2), round(ze, 2)), fontsize= 'xx-large')

myTarget = ax.text2D(0, 0, "Target: X(0), Y(0), Z(0)", fontsize= 'xx-large', transform=ax.transAxes)

# Use "exitFlag" to halt while loop execution and terminate script.
exitFlag = False
mode=1

def on_key_press(event):
    '''Key press event to stop script execution if Enter is pressed,
        or toggle mode if Shift is pressed.
    '''
    global exitFlag, mode
    if event.key == 'enter':
        exitFlag = True
    elif event.key == 'shift':
        mode *= -1

fig.canvas.mpl_connect('key_press_event', on_key_press)

# Turn on interactive plotting and show plot.
plt.ion()
plt.show()

x1 = 0
y1 = 0
z1 = 0

while not exitFlag:
    if mode == -1:
        x1 = random.randint(-10, 10)
        y1 = random.randint(-20, -5)
        z1 = random.randint(5, 20)
        #z1 = 10
        print("Target: {}, {}, {}".format(x1, y1, z1))
        myRobot.set_target(x1, y1, z1)
        mode = 1
    if not myRobot.on_target():
        myRobot.move_to_target()
        x2, y2, _ = proj3d.proj_transform([myRobot.end_position['target'].x], [myRobot.end_position['target'].y], [ze], ax.get_proj())
        targetPt.set_data((x2, y2))
        armPlot.set_data((myRobot.joints[0, :], myRobot.joints[1, :]), myRobot.joints[2, :])
        # Update joint angles
        for i in range(myRobot.joint_count):
            xr = myRobot.joints[0, i]
            yr =  myRobot.joints[1, i]
            zr = myRobot.joints[2, i]
            rot = math.degrees(myRobot.theta['theta'][i])
            joint_labels[i].set_text("Joint {}: X({}), Y({}), Z({}), Rot({})".format(
                        i,round(xr, 2),round(yr, 2),round(zr, 2),round(rot, 2)))
            x2, y2, _ = proj3d.proj_transform([xr], [yr], [zr], ax.get_proj())
            joint_labels[i].set_position((x2, y2))
        # Set label for end effector
        xe = myRobot.joints[0, -1]
        ye =  myRobot.joints[1, -1]
        ze = myRobot.joints[2, -1]
        effector.set_text("End Effector: X({}), Y({}), Z({})".format(
                            round(xe, 2),round(ye, 2), round(ze, 2)))
        x2, y2, _ = proj3d.proj_transform([xe], [ye], [ze], ax.get_proj())
        effector.set_position((x2,y2))
        myTarget.set_text("Target: X({}), Y({}), Z({})".format(
                            round(myRobot.end_position['target'].x, 2),
                            round(myRobot.end_position['target'].y, 2),
                            round(myRobot.end_position['target'].z, 2)))
    fig.canvas.get_tk_widget().update()
