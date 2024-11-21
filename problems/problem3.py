from typing import Tuple, List
from colorama import Fore, Style
import numpy as np

import sys 
import os

#print(os.path.abspath(os.path.join(os.path.dirname(__file__), '../')))
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../')))
from robot.kinematics import robot_ik, robot_fk, JointState, CartesianGoal, DEFAULT_PARAMS

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Circle parameters
R = 0.032  # Radius in meters
xc, yc, zc = 0.15, 0.0, 0.12  # Center of the circle in meters

# Generate 36 equidistant points on the circle in the YZ-plane
num_points = 36
phis = np.linspace(0, 2 * np.pi, num_points)
points = [CartesianGoal(x=xc, y=yc + R * np.cos(phi), z=zc + R * np.sin(phi), rz=0) for phi in phis]

# Display the numerical results of tracking
print("Numerical Results of Robot Arm Tracking:\n")
for i, point in enumerate(points):
    joint_states = robot_ik(point)
    if joint_states:
        js = joint_states[0]  # Use the first valid joint state
        print(f"Point {i + 1}: Cartesian Goal = (x: {point.x:.3f}, y: {point.y:.3f}, z: {point.z:.3f})")
        print(f"  Joint Angles: q1 = {js.q1:.3f}, q2 = {js.q2:.3f}, q3 = {js.q3:.3f}, q4 = {js.q4:.3f}\n")

# Plotting the 3D circle and the robot arm positions
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')
ax.plot([p.x for p in points], [p.y for p in points], [p.z for p in points], 'b--', label="Desired Circle")


# Compute and plot the arm configuration for each point
for point in points:
    joint_states = robot_ik(point)
    for js in joint_states:
        x = [0]
        y = [0]
        z = [0]
        for idx in range(1, 5):
            P = robot_fk(js, 0, idx)
            x.append(P[0, 3])
            y.append(P[1, 3])
            z.append(P[2, 3])
        # Plot the robot arm
        ax.plot(x, y, z, 'ro-')
        break

# Set plot limits and labels
ax.set_xlim(-0.2, 0.2)
ax.set_ylim(-0.2, 0.2)
ax.set_zlim(0, 0.3)
ax.set_xlabel("X-axis (m)")
ax.set_ylabel("Y-axis (m)")
ax.set_zlabel("Z-axis (m)")
ax.set_title("3D Robot Arm Tracking a Circle with Integrated Inverse Kinematics")
ax.legend()
plt.show()