#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt

obs = np.load('gain_1_y_1_traj.npy')
x, y = obs[:, 0], obs[:, 1]
plt.figure()
plt.title("The affect of the wave-gain on the robot position")
plt.plot(x, y)
# plt.axis('equal')
plt.show()