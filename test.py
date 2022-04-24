#!/usr/bin/env python3

import numpy as np
import math, random
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from revolute_robot import RevoluteRobot

myRobot = RevoluteRobot()

myRobot.add_joint_link(length=9.6,  min_theta= 0,   max_theta=180)
myRobot.add_joint_link(length=10.5, min_theta= 0,   max_theta=180)
myRobot.add_joint_link(length=10.2, min_theta= 210, max_theta=360)
myRobot.add_joint_link(length=11.0, min_theta= 270, max_theta=360)

fig = plt.figure(figsize=(10, 10))
ax = fig.add_subplot(111, projection='3d')

#ax.plot([0, 3], [0, 6], [0, 10])
ax.plot(myRobot.joints[0, :], myRobot.joints[1, :], myRobot.joints[2, :], marker='o', c='g', lw=2)

ax.set_xlabel('$X$', fontsize=20)
ax.set_ylabel('$Y$', fontsize=20)
ax.set_zlabel('$Z$', fontsize=20)

plt.show()
