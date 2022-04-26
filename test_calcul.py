#!/usr/bin/env python3

import math
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d import proj3d

joints = np.array([[0, 0, 0, 1]], dtype=np.float).T

for i in range(4):
    joints = np.append(joints, np.array([[0, 0, 0, 1]]).T, axis=1)

t1 = math.radians(90)
t2 = math.radians(45)
t3 = math.radians(270)
t4 = math.radians(300)

a1 = 10
a2 = 10
a3 = 10
a4 = 5

T = np.array([
    [math.cos(t1), 0,  math.sin(t1),   0],
    [math.sin(t1), 0, -math.cos(t1),   0],
    [           0, 1,             0,  a1],
    [           0, 0,             0,   1]])

joints[:, [1]] = T.dot(np.array([[0, 0, 0, 1]]).T)

T_next = np.array([
    [math.cos(t2), -math.sin(t2),  0,  a2*math.cos(t2)],
    [math.sin(t2),  math.cos(t2),  0,  a2*math.sin(t2)],
    [           0,             0,  1,  0],
    [           0,             0,  0,  1]])

T = T.dot(T_next)
joints[:, [2]] = T.dot(np.array([[0, 0, 0, 1]]).T)

T_next = np.array([
    [math.cos(t3), -math.sin(t3),  0,  a3*math.cos(t3)],
    [math.sin(t3),  math.cos(t3),  0,  a3*math.sin(t3)],
    [           0,             0,  1,  0],
    [           0,             0,  0,  1]])

T = T.dot(T_next)
joints[:, [3]] = T.dot(np.array([[0, 0, 0, 1]]).T)

T_next = np.array([
    [math.cos(t4), -math.sin(t4),  0,  a4*math.cos(t4)],
    [math.sin(t4),  math.cos(t4),  0,  a4*math.sin(t4)],
    [           0,             0,  1,  0],
    [           0,             0,  0,  1]])

T = T.dot(T_next)
joints[:, [4]] = T.dot(np.array([[0, 0, 0, 1]]).T)

print(joints)

end_position = np.array([[0, 0, 0, 1]]).T


fig = plt.figure(figsize=(20, 20))
ax = fig.add_subplot(111, projection='3d')
fig.subplots_adjust(left=0.05, bottom=0.05, right=1, top=1)

armPlot, = ax.plot(joints[0, :], joints[1, :], joints[2, :], marker='o', c='g', lw=2)

ax.set_xlabel('$X$', fontsize=20)
ax.set_ylabel('$Y$', fontsize=20)
ax.set_zlabel('$Z$', fontsize=20)

ax.set_xlim(-30, 30)
ax.set_ylim(-30, 30)
ax.set_zlim(0, 30)

#plt.ion()
plt.show()
