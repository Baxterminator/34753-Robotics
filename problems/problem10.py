import numpy as np
import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../')))

from robot.kinematics import force_zero, robot_ik, robot_fk, jacobians, joints_velocities,joints_velocities2, quintic, JointState, CartesianGoal, DEFAULT_PARAMS
from robot.dynamics import expanded_mass



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

# position of the center of mass of each link w.r.t the reference frame of the link
COM = [
    CartesianGoal(x=0,y=-20,z=0),
    CartesianGoal(x=-30,y=0,z=0),
    CartesianGoal(x=-30,y=0,z=0),
    CartesianGoal(x=-25,y=15,z=0),
]


# masses 
m1 = 60e-3 # Kg
m2 = 80e-3 # kg
m3 = 80e-3 # kg
m4 = 40e-3 # kg
# inertia tensors
I0 = 2.3386e-5 # Kg m^2
I1 = [[I0,     0,      0],
      [0,      0.4*I0, 0],
      [0,      0,      0.9*I0]]
I2 = [[0.45*I0,0,      0],
      [0,      1.4*I0, 0],
      [0,      0,      1.2*I0]]
I3 = I2
I4 = [[0.5*I0, 0,      0],
      [0,      0.5*I0, 0],
      [0,      0,      0.5*I0]]

# mass matrix
M = np.eye(24)
for i in range(3):
    M[i,i] = m1
    M[i + 6,i + 6] = m2
    M[i + 12,i + 12] = m3
    M[i + 18,i + 18] = m4
    M[i + 3,i + 3] = I1[i][i]
    M[i + 9,i + 9] = I2[i][i]
    M[i + 15,i + 15] = I3[i][i]
    M[i + 21,i + 21] = I4[i][i]
    

# Compute and plot the arm configuration for each point
for i in range(len(points)):
    
    joint_states = robot_ik(points[i])    
    # compute the jacobian for the first solution:
    D = expanded_mass(joint_states[0],COM,M)
    
    print(D.shape)
    
