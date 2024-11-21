from typing import Tuple, List

import numpy as np

from .types import RobotParameters, CartesianGoal, DEFAULT_PARAMS, JointState, T45


def divide_by_cos_or_sin(value_cos: float, value_sin: float, theta: float) -> float:
    c = np.cos(theta)
    if np.fabs(c) < 1E-6:
        return -value_sin / np.sin(theta)
    return -value_cos / c


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
        
