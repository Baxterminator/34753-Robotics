from typing import Tuple, List
# from colorama import Fore, Style
import numpy as np

import sys 
import os

import time

#print(os.path.abspath(os.path.join(os.path.dirname(__file__), '../')))
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../')))
from robot.kinematics import force_zero, robot_ik, robot_fk, jacobians, joints_velocities,joints_velocities2, quintic, JointState, CartesianGoal, DEFAULT_PARAMS

# import matplotlib.pyplot as plt
# from mpl_toolkits.mplot3d import Axes3D

import dynamixel_sdk as dxl

# parameters for the servos
ADDR_MX_TORQUE_ENABLE = 24
ADDR_MX_CW_COMPLIANCE_MARGIN = 26
ADDR_MX_CCW_COMPLIANCE_MARGIN = 27
ADDR_MX_CW_COMPLIANCE_SLOPE = 28
ADDR_MX_CCW_COMPLIANCE_SLOPE = 29
ADDR_MX_GOAL_POSITION = 30
ADDR_MX_MOVING_SPEED = 32
ADDR_MX_PRESENT_POSITION = 36
ADDR_MX_PUNCH = 48
PROTOCOL_VERSION = 1.0
DXL_IDS = [1, 2, 3, 4]
DEVICENAME = "COM9"
BAUDRATE = 1000000
TORQUE_ENABLE = 1
TORQUE_DISABLE = 0
portHandler = dxl.PortHandler(DEVICENAME)
packetHandler = dxl.PacketHandler(PROTOCOL_VERSION)
portHandler.openPort()
portHandler.setBaudRate(BAUDRATE)

def rotate_rest(angles: List = [0,0,0,0]):
    return [angles[0] + 150,
            angles[1] + 150,
            angles[2] + 150,
            angles[3] + 150]

# first go to resting position
for DXL_ID in [1,2,3]:
    packetHandler.write1ByteTxRx(
        portHandler, DXL_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE
    )
    packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_MX_CW_COMPLIANCE_MARGIN, 0)
    packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_MX_CCW_COMPLIANCE_MARGIN, 0)
    packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_MX_CW_COMPLIANCE_SLOPE, 32)
    packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_MX_CCW_COMPLIANCE_SLOPE, 32)
    packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_MX_MOVING_SPEED, 60)
    packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_MX_GOAL_POSITION, 508)
    
angle1 = int(150/300*1023)
packetHandler.write1ByteTxRx(
        portHandler, 4, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE
    )
# packetHandler.write2ByteTxRx(portHandler, 1, ADDR_MX_CW_COMPLIANCE_MARGIN, 0)
# packetHandler.write2ByteTxRx(portHandler, 1, ADDR_MX_CCW_COMPLIANCE_MARGIN, 0)
# packetHandler.write1ByteTxRx(portHandler, 1, ADDR_MX_CW_COMPLIANCE_SLOPE, 32)
# packetHandler.write1ByteTxRx(portHandler, 1, ADDR_MX_CCW_COMPLIANCE_SLOPE, 32)
packetHandler.write2ByteTxRx(portHandler, 4, ADDR_MX_MOVING_SPEED, 60)
packetHandler.write2ByteTxRx(portHandler, 4, ADDR_MX_GOAL_POSITION,angle1)

time.sleep(5)

# Circle parameters
R = 0.032  # Radius in meters
xc, yc, zc = 0.15, 0.0, 0.12  # Center of the circle in meters

#phis = [0, np.pi/2, np.pi, np.pi*3/2, 2*np.pi]
phis = [0,np.pi*(0+1/6), np.pi/2, np.pi, np.pi*3/2,np.pi*(2-1/6), 2*np.pi]
points = [CartesianGoal(x=xc, y=yc + R * np.cos(phi), z=zc + R * np.sin(phi), rz=0) for phi in phis]


# module of the velocity is constant. change only the direction
v_lin_module = 0.0027
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

t = 0

# state of the interpolated points
q = JointState
# velocity of joints
#qd_js = JointState

# Plotting the 3D circle 
phis_ideal = np.linspace(0, 2 * np.pi, 36)
points_ideal = [CartesianGoal(x=xc, y=yc + R * np.cos(phi), z=zc + R * np.sin(phi), rz=0) for phi in phis_ideal]

# fig = plt.figure(figsize=(10, 8))
# ax = fig.add_subplot(111, projection='3d')
# ax.plot([p.x for p in points_ideal], [p.y for p in points_ideal], [p.z for p in points_ideal], 'b--', label="Desired Circle")

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
    print(f"joint_velocities:\n {qd1}")
    print(f"velocities:\n {force_zero(J[0]@qd1)} ")
    
    
    if(i!=0):
        seq_q1, qd_1, qdd_1 = quintic(js0.q1,js.q1,qd0[0][0],qd[0][0],t,t+2)
        seq_q2, qd_2, qdd_2 = quintic(js0.q2,js.q2,qd0[1][0],qd[1][0],t,t+2)
        seq_q3, qd_3, qdd_3 = quintic(js0.q3,js.q3,qd0[2][0],qd[2][0],t,t+2)
        seq_q4, qd_4, qdd_4 = quintic(js0.q4,js.q4,qd0[3][0],qd[3][0],t,t+2)
        #print(seq_q1)
        
        for s in range(len(seq_q1)):
            # convert radiants in degrees
            angles = rotate_rest([np.degrees(seq_q1[s]),
                      np.degrees(seq_q2[s]),
                      np.degrees(seq_q3[s]),
                      np.degrees(seq_q4[s])])

            # # convert rad/s in deg/s
            # speeds = [np.degrees(qd_1[s]),
            #           np.degrees(qd_2[s]),
            #           np.degrees(qd_3[s]),
            #           np.degrees(qd_4[s])]
            
            for DXL_ID in DXL_IDS:
                # Convert angle to position value for the AX-12A
                # Target angles for servos (0° to 300°)
                dxl_goal_position = int((angles[DXL_ID-1] / 300.0) * 1023)
                
                # Convert speed to speed value for the AX-12A
                # dxl_speed = int((speeds[DXL_ID-1] / 354.0) * 1023)
                
                packetHandler.write1ByteTxRx(
                    portHandler, DXL_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE
                )
                packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_MX_CW_COMPLIANCE_MARGIN, 0)
                packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_MX_CCW_COMPLIANCE_MARGIN, 0)
                packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_MX_CW_COMPLIANCE_SLOPE, 32)
                packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_MX_CCW_COMPLIANCE_SLOPE, 32)
                packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_MX_MOVING_SPEED, 90)
                packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_MX_GOAL_POSITION, dxl_goal_position)
                
            time.sleep(0.25)
    
    js0 = js
    qd0 = qd
    

    
    
    
    



        