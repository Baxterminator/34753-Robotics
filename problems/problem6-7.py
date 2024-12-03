from typing import Tuple, List
from colorama import Fore, Style
import numpy as np

import sys 
import os


#print(os.path.abspath(os.path.join(os.path.dirname(__file__), '../')))
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../')))
from robot.kinematics import force_zero, robot_ik, robot_fk, jacobians, joints_velocities,joints_velocities2, quintic, JointState, CartesianGoal, DEFAULT_PARAMS

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


# Circle parameters
R = 0.032  # Radius in meters
xc, yc, zc = 0.15, 0.0, 0.12  # Center of the circle in meters

#phis = [0, np.pi/2, np.pi, np.pi*3/2, 2*np.pi]
phis = [0,np.pi*(0+1/6), np.pi/2, np.pi, np.pi*3/2,np.pi*(2-1/6), 2*np.pi]
points = [CartesianGoal(x=xc, y=yc + R * np.cos(phi), z=zc + R * np.sin(phi), rz=0) for phi in phis]


# module of the velocity is constant. change only the direction
v_lin_module = 0.027
# v_lin = [np.array([0,0,0])*v_lin_module, 
#          np.array([0,-1,0])*v_lin_module,
#          np.array([0,0,-1])*v_lin_module,
#          np.array([0,1,0])*v_lin_module,
#          np.array([0,0,0])*v_lin_module]

v_lin = []
for phi in phis:
    if phi !=0 and phi!=2*np.pi:
        v_lin.append(force_zero(np.array([0,-np.sin(phi),np.cos(phi)]))*v_lin_module)
    else:
        v_lin.append(np.array([0,0,0]))
        
#print(v_lin)

time = 0

# state of the interpolated points
q = JointState

# Plotting the 3D circle 
phis_ideal = np.linspace(0, 2 * np.pi, 36)
points_ideal = [CartesianGoal(x=xc, y=yc + R * np.cos(phi), z=zc + R * np.sin(phi), rz=0) for phi in phis_ideal]

fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')
ax.plot([p.x for p in points_ideal], [p.y for p in points_ideal], [p.z for p in points_ideal], 'b--', label="Desired Circle")

# Compute and plot the arm configuration for each point
for i in range(len(points)):
    
    joint_states = robot_ik(points[i])    
    # compute the jacobian for the first solution:
    for js in joint_states:
        # homogeneous matrices of the joints w.r.t the base frame
        T = []
        for idx in range(1, 5):
            T.append(robot_fk(js, 0, idx)) 
            
        break
    print(len(T))
    J = jacobians(T)
    
    # compute the joints velocity vector:
    r = force_zero(np.array([points[i].x-xc, points[i].y-yc, points[i].z-zc]))
    qd = joints_velocities2(v_lin[i],J[0],js)
    qd1 = joints_velocities(v_lin[i],r,J[0])
    print(f"joint_velocities2:\n {qd}")
    print(f"joint_angle:\n {js}")
    print(f"velocities:\n {force_zero(J[0]@qd1)} ")
    
    
    if(i!=0):
        seq_q1, qd_4, qdd_4 = quintic(js0.q1,js.q1,qd0[0][0],qd[0][0],time,time+2)
        seq_q2, qd_4, qdd_4 = quintic(js0.q2,js.q2,qd0[1][0],qd[1][0],time,time+2)
        seq_q3, qd_4, qdd_4 = quintic(js0.q3,js.q3,qd0[2][0],qd[2][0],time,time+2)
        seq_q4, qd_4, qdd_4 = quintic(js0.q4,js.q4,qd0[3][0],qd[3][0],time,time+2)
        #print(seq_q1)
        
        for s in range(len(seq_q1)):
            q.q1 = seq_q1[s]
            q.q2 = seq_q2[s]
            q.q3 = seq_q3[s]
            q.q4 = seq_q4[s]
            x = []
            y = []
            z = []
            #for idx in range(1, 5):
            P = robot_fk(q, 0)
            x.append(P[0, 3])
            y.append(P[1, 3])
            z.append(P[2, 3])
            # Plot the robot arm
            ax.plot(x, y, z, 'ro-')
    
    js0 = js
    qd0 = qd
    
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
    
    
    
    



        