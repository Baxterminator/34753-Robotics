import numpy as np
import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../')))

from robot.kinematics import force_zero, robot_ik, robot_fk, jacobians, joints_velocities,joints_velocities2, quintic, JointState, CartesianGoal, DEFAULT_PARAMS, T45
from robot.dynamics import expanded_mass

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
DEVICENAME = ""
BAUDRATE = 1000000
TORQUE_ENABLE = 1
TORQUE_DISABLE = 0
portHandler = dxl.PortHandler(DEVICENAME)
packetHandler = dxl.PacketHandler(PROTOCOL_VERSION)
portHandler.openPort()
portHandler.setBaudRate(BAUDRATE)

# computer vision part for recognizing the line 

# discretize the line in sample points

# get the real world coordinate of each point w.r.t image frame
points_l = [] # list of list of 2 points in local frame coordinates
js = JointState()  # jointstates in which picture was taken

# apply robot forward kinematic 
T04 = robot_fk(js)
T05= T04 @ T45

#list of global ref coordinates points
points = [] 
# rotate from image frame to camera frame

for point_l in points_l:
    # compute the global point
    p = np.linalg.inv(T05) @ point_l[0]
    p_neigh = np.linalg.inv(T05) @ point_l[1]
    # add 10 mm to the z axis to avoid collision with the table
    points.append([CartesianGoal(x=p[0],y=p[1],z=p[2]+10),
                   CartesianGoal(x=p_neigh[0],y=p_neigh[1],z=p_neigh[2]+10)])
    
    
# keep constant linear speed during the path
v_lin_mod = 0.027 #m/s

# compute the velocity direction vector for each point
v_lin = []
for p in points:
    v = p[0]-p[1]
    v_dir = v/np.linalg.norm(v)
    v_lin.append(v_dir*v_lin_mod)
    


# interpolation tume
time = 0
# state of the interpolated points
q = JointState

for i in range(len(points)):
    
    joint_states = robot_ik(points[i][0])    
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
    qd = joints_velocities2(v_lin[i],J[0],js)
    #qd = joints_velocities(v_lin[i],r,J[0])

    
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
            
    
    js0 = js
    qd0 = qd
    

    
    
    
    



        
    

    
