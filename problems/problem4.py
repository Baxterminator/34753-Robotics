from typing import Tuple, List
from colorama import Fore, Style
import numpy as np

import sys 
import os

# Circle parameters
R = 0.032  # Radius in meters
xc, yc, zc = 0.15, 0.0, 0.12  # Center of the circle in meters

#print(os.path.abspath(os.path.join(os.path.dirname(__file__), '../')))
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../')))
from robot.kinematics import robot_ik, robot_fk, jacobians, JointState, CartesianGoal, DEFAULT_PARAMS

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

phis = [0, np.pi/2, np.pi, np.pi*3/2, 2*np.pi]
points = [CartesianGoal(x=xc, y=yc + R * np.cos(phi), z=zc + R * np.sin(phi), rz=0) for phi in phis]


# Plotting the 3D circle and the robot arm positions
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')
ax.plot([p.x for p in points], [p.y for p in points], [p.z for p in points], 'b--', label="Desired Circle")

# Compute and plot the arm configuration for each point
for point in points:
    joint_states = robot_ik(point)
    # there are many soluions but we just take the first one
    for js in joint_states:
        # homogeneous matrices of the joints w.r.t the base frame
        T = []
        for idx in range(1, 5):
            T.append(robot_fk(js, 0, idx)) 
            
        break
    
    J = jacobians(T)
    print(f"joint state values: {js} \n\n")
    print(f"Jacobian J4: \n\n {J[0]} \n\n Jacobian J5:\n\n {J[0]}\n")
        



        