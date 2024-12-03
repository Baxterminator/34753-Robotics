from typing import Tuple, List

import numpy as np

from .types import RobotParameters, CartesianGoal, DEFAULT_PARAMS, JointState, T45


def divide_by_cos_or_sin(value_cos: float, value_sin: float, theta: float) -> float:
    c = np.cos(theta)
    if np.fabs(c) < 1E-6:
        return -value_sin / np.sin(theta)
    return -value_cos / c


def translation_matrix(goal: CartesianGoal) -> np.ndarray:
    H = np.eye(4)
    H[0,3] = goal.x
    H[1,3] = goal.y
    H[2,3] = goal.z
    return H


def dh_matrix(th: float, d: float, alpha: float, a: float) -> np.ndarray:
    return np.array([
        [np.cos(th), -np.sin(th) * np.cos(alpha), np.sin(th) * np.sin(alpha), a * np.cos(th)],
        [np.sin(th), np.cos(th) * np.cos(alpha), -np.cos(th) * np.sin(alpha), a * np.sin(th)],
        [0, np.sin(alpha), np.cos(alpha), d],
        [0, 0, 0, 1]
    ])


def force_zero(m: np.ndarray, min_number=1E-8) -> np.ndarray:
    mask = np.bitwise_and((-min_number < m), (m < min_number))
    m[mask] = 0
    return m


def normalize_angle(a: float) -> float:
    return np.mod(a + np.pi, 2 * np.pi) - np.pi

# to compute homogeneous matrix with respect to base from for all the joints
def robot_fk(js: JointState, fr: int = 0, to: int = 4, robot: RobotParameters = DEFAULT_PARAMS) -> np.ndarray:
    matrices = [
        dh_matrix(js.q1, robot.d1, np.deg2rad(90), 0),
        dh_matrix(js.q2 + np.deg2rad(90), 0, 0, robot.a2),
        dh_matrix(js.q3, 0, 0, robot.a3),
        dh_matrix(js.q4, 0, 0, robot.a4),
    ]
    M = np.eye(4)
    for idx in range(fr, to):
        M = M @ matrices[idx]
    return force_zero(M)


def robot_ik(goal: CartesianGoal, robot: RobotParameters = DEFAULT_PARAMS) -> List[JointState]:
    def compute_theta_1() -> List[JointState]:
        th1_1 = 0
        th1_2 = 0
        if goal.x != 0 or goal.y != 0:
            xy = goal.x * goal.x + goal.y * goal.y
            th1_1 = 0.5 * np.arctan2(2 * goal.x * goal.y / xy, (goal.x * goal.x - goal.y * goal.y) / xy)
            th1_2 = normalize_angle(th1_1 + np.pi)
        return [
            JointState(th1_1, 0, 0, 0),
            JointState(th1_2, 0, 0, 0),
        ]

    def compute_a(_js: JointState) -> float:
        return divide_by_cos_or_sin(goal.x, goal.y, _js.q1)

    def compute_theta_3(_js: JointState, _A: float, _th24: float) -> List[JointState]:
        D1 = _A - robot.a4 * np.sin(_th24)
        D2 = goal.z - robot.d1 - robot.a4 * goal.rz
        S = D1 * D1 + D2 * D2
        cth3 = (S - robot.a2 * robot.a2 - robot.a3 * robot.a3) / (2 * robot.a2 * robot.a3)
        if np.fabs(cth3) <= 1:
            #print(f"cos(th3)={cth3}")
            th3 = np.arccos(cth3)
            return [
                JointState(_js.q1, 0, th3, 0),
                JointState(_js.q1, 0, -th3, 0),
            ]
        return []

    def compute_theta_2(_js: JointState, _A: float, _th24: float) -> List[JointState]:
        d1 = _A - robot.a4 * np.sin(_th24)
        d2 = goal.z - robot.d1 - robot.a4 * goal.rz
        s = d1 * d1 + d2 * d2
        k1 = robot.a2 + robot.a3 * np.cos(_js.q3)
        k2 = robot.a3 * np.sin(_js.q3)
        th2 = np.arctan2((k1 * d1 - k2 * d2) / s, (k2 * d1 + k1 * d2) / s)
        return [JointState(_js.q1, th2, _js.q3, 0)]

    def compute_theta_4(_js: JointState, _th24: float) -> JointState:
        return JointState(
            _js.q1,
            _js.q2,
            _js.q3,
            normalize_angle(_th24 - _js.q2 - _js.q3)
        )

    # ==============================
    # Compute solutions
    # ==============================

    # Compute theta 1
    sol_th1 = compute_theta_1()
    A = [compute_a(_js) for _js in sol_th1]
    th24 = np.arccos(goal.rz)

    # Compute theta 3
    sol_th3 = []
    A_th3 = []
    th24_th3 = []
    for i in range(len(sol_th1)):
        # + Theta 24
        f = compute_theta_3(sol_th1[i], A[i], th24)
        sol_th3.extend(f)
        A_th3.extend([A[i] for k in range(len(f))])
        th24_th3.extend([th24 for k in range(len(f))])

        # - Theta 24
        f = compute_theta_3(sol_th1[i], A[i], -th24)
        sol_th3.extend(f)
        A_th3.extend([A[i] for k in range(len(f))])
        th24_th3.extend([-th24 for k in range(len(f))])

    # Compute theta 2
    solutions = []
    for i in range(len(sol_th3)):
        solutions.extend(compute_theta_2(sol_th3[i], A_th3[i], th24_th3[i]))

    # Compute theta 4
    solutions = [compute_theta_4(solutions[i], th24_th3[i]) for i in range(len(solutions))]

    return solutions


def jacobians(T: list) -> List[np.ndarray]:
        
    #define the matrix T05   
    T05 = T[-1] @ T45
    
    # define the origins and the z vector
    o = []
    z = []
    o.append(np.array([0,0,0]))
    z.append(np.array([0,0,1]))
    for t in T:
        o.append(t[0:3,3])
        z.append(t[0:3,2])
        
    o.append(T05[0:3,3])
    
    # compute the jacobians
    # for end effector: J4
    # and for the camera: J5
    
    for idx in range(0,4):        
        j4 = np.concatenate((np.cross(z[idx],(o[-2] - o[idx])).reshape(-1,1),z[idx].reshape(-1,1)), axis=0)
        
        # add columns to J4
        try:
            J4 = np.hstack((J4, j4))  
        except NameError:
            J4 = j4  # if J does not exist create it
        
        j5 = np.concatenate((np.cross(z[idx],(o[-1] - o[idx])).reshape(-1,1),z[idx].reshape(-1,1)), axis=0)
        
        # add columns to J4
        try:
            J5 = np.hstack((J5, j5))  
        except NameError:
            J5 = j5  # if J does not exist create it
                
    return [J4,J5]


def joints_velocities(v_lin: np.ndarray, r: np.ndarray, J: np.ndarray) -> np.ndarray:
    
    v_norm = np.linalg.norm(v_lin)
    if v_norm != 0:
        r_norm = np.linalg.norm(r)
        
        w_norm = v_norm/r_norm
        w_dir = np.cross(r/r_norm,v_lin/v_norm)
        w = w_dir*w_norm
        
        v =  np.concatenate((v_lin.reshape(-1,1),w.reshape(-1,1)))
        # pseudoinverse of J @ v
        J_plus = np.linalg.inv(np.transpose(J) @ J) @ np.transpose(J) 
        return J_plus @ v
        
    else:
        # v = [0,0,0,0,0,0]'
        v = np.concatenate((v_lin.reshape(-1,1),v_lin.reshape(-1,1)))
        #print(v)
        return np.array([0,0,0,0]).reshape(-1,1)
        
  
def joints_velocities2(v_lin: np.ndarray,J: np.ndarray,js: JointState) -> np.ndarray: 
    v_norm = np.linalg.norm(v_lin)
    if v_norm != 0:
        
        
        v =  np.concatenate((v_lin.reshape(-1,1),np.array([[0]])))
        # define linear velocity jacpobian:
        Jv = J[0:3,:]
        
        J_plus =  np.concatenate((Jv,np.array([[0,-np.sin(js.q2+js.q3+js.q4),-np.sin(js.q2+js.q3+js.q4),-np.sin(js.q2+js.q3+js.q4)]])))
        return np.linalg.inv(J_plus) @ v
        
    else:
        # v = [0,0,0,0,0,0]'
        v = np.concatenate((v_lin.reshape(-1,1),v_lin.reshape(-1,1)))
        #print(v)
        return np.array([0,0,0,0]).reshape(-1,1)


def quintic(q0: float, q1: float, qd0: float, qd1: float,t0: float, t1:float, qdd0: float = 0.0, qdd1: float = 0.0, n_points: int = 10):
    
    # q0 = initial position
    # qd0 = initial velocity
    # qdd0 = initial acceleration
    # q1 = final position
    # qd1 = final velocity
    # qdd1 = final acceleration
    # t0 = initial time
    # t1 = final time
    
    t = np.linspace(t0, t1, n_points * (t1 - t0))
    
    # coefficient matrix for quintic system
    M = np.array([
        [1, t0, t0**2, t0**3, t0**4, t0**5],
        [0, 1, 2*t0, 3*t0**2, 4*t0**3, 5*t0**4],
        [0, 0, 2, 6*t0, 12*t0**2, 20*t0**3],
        [1, t1, t1**2, t1**3, t1**4, t1**5],
        [0, 1, 2*t1, 3*t1**2, 4*t1**3, 5*t1**4],
        [0, 0, 2, 6*t1, 12*t1**2, 20*t1**3]
    ])

    # known values
    b = np.array([q0, qd0, qdd0, q1, qd1, qdd1])

    # solve linear system
    a = np.linalg.solve(M, b)

    # qd = position trajectory
    # vd = velocity trajectory
    # ad = acceleration trajectory
    c = np.ones_like(t)
    q = (a[0]*c + a[1]*t + a[2]*t**2 + a[3]*t**3 + a[4]*t**4 + a[5]*t**5)
    qd = (a[1]*c + 2*a[2]*t + 3*a[3]*t**2 + 4*a[4]*t**3 + 5*a[5]*t**4)
    qdd = (2*a[2]*c + 6*a[3]*t + 12*a[4]*t**2 + 20*a[5]*t**3)

    return q, qd, qdd
