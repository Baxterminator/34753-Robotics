from typing import List

import numpy as np

from robotics.types import RobotParameters, CartesianGoal, DEFAULT_PARAMS, JointState, Point3D
from robotics.math import divide_by_cos_or_sin, force_zero, normalize_angle


class MatrixGen:
    @staticmethod
    def dh_matrix(th: float, d: float, alpha: float, a: float) -> np.ndarray:
        return np.array([
            [np.cos(th), -np.sin(th) * np.cos(alpha), np.sin(th) * np.sin(alpha), a * np.cos(th)],
            [np.sin(th), np.cos(th) * np.cos(alpha), -np.cos(th) * np.sin(alpha), a * np.sin(th)],
            [0, np.sin(alpha), np.cos(alpha), d],
            [0, 0, 0, 1]
        ])

    @staticmethod
    def translation_matrix(t: Point3D):
        return np.array([
            [1, 0, 0, t.x],
            [0, 1, 0, t.y],
            [0, 0, 1, t.z],
            [0, 0, 0, 1]
        ])


def robot_fk(js: JointState, fr: int = 0, to: int = 4, robot: RobotParameters = DEFAULT_PARAMS) -> np.ndarray:
    matrices = [
        MatrixGen.dh_matrix(js.q1, robot.d1, np.deg2rad(90), 0),
        MatrixGen.dh_matrix(js.q2 + np.deg2rad(90), 0, 0, robot.a2),
        MatrixGen.dh_matrix(js.q3, 0, 0, robot.a3),
        MatrixGen.dh_matrix(js.q4, 0, 0, robot.a4),
        MatrixGen.translation_matrix(Point3D(robot.dx_c, robot.dy_c, 0))
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
            # print(f"cos(th3)={cth3}")
            th3 = np.arccos(cth3)
            return [
                JointState(_js.q1, 0, th3, 0),
                JointState(_js.q1, 0, -th3, 0),
            ]
        return []

    def compute_theta_2(_js: JointState, _A: float, _th24: float) -> JointState:
        d1 = _A - robot.a4 * np.sin(_th24)
        d2 = goal.z - robot.d1 - robot.a4 * goal.rz
        s = d1 * d1 + d2 * d2
        k1 = robot.a2 + robot.a3 * np.cos(_js.q3)
        k2 = robot.a3 * np.sin(_js.q3)
        th2 = np.arctan2((k1 * d1 - k2 * d2) / s, (k2 * d1 + k1 * d2) / s)
        return JointState(_js.q1, th2, _js.q3, 0)

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
    solutions = []
    th24 = np.arccos(goal.rz)
    for js_th1 in compute_theta_1():
        A = compute_a(js_th1)
        for js_th3 in compute_theta_3(js_th1, A, th24):
            solutions.append(compute_theta_4(compute_theta_2(js_th3, A, th24), th24))
        for js_th3 in compute_theta_3(js_th1, A, -th24):
            solutions.append(compute_theta_4(compute_theta_2(js_th3, A, -th24), -th24))
    return solutions


def jacobian(joint_state: JointState, link_to: int = 4, robot: RobotParameters = DEFAULT_PARAMS) -> np.ndarray:
    J = np.zeros((6, link_to))
    Tn = robot_fk(joint_state, 0, link_to, robot)

    # Construct each column
    # Since all joints are revolute, they all follow the same formula
    for idx in range(1, link_to + 1):
        T = robot_fk(joint_state, 0, idx - 1, robot)

        # Computing velocity jacobian
        # Jv = z(i-1) x (o(n) - o(i-1))
        J[:3, idx - 1] = np.cross(T[0:3, 2], Tn[0:3, 3] - T[0:3, 3])

        # Computing angular jacobian
        # Jw = z(i-1)
        J[3:, idx - 1] = T[0:3, 2]
    return J


def joints_velocities(v_lin: np.ndarray, r: np.ndarray, J: np.ndarray) -> JointState:
    v_norm = np.linalg.norm(v_lin)
    if v_norm != 0:
        r_norm = np.linalg.norm(r)

        w_norm = v_norm / r_norm
        w_dir = np.cross(r / r_norm, v_lin / v_norm)
        w = w_dir * w_norm

        v = np.concatenate((v_lin.reshape(-1, 1), w.reshape(-1, 1)))
        # pseudoinverse of J @ v
        J_plus = np.linalg.inv(np.transpose(J) @ J) @ np.transpose(J)
        return JointState.from_vector(J_plus @ v)

    else:
        return JointState(0, 0, 0, 0)


# Wrapping class
class RobotModel:
    def __init__(self, robot_params: RobotParameters = DEFAULT_PARAMS):
        self.params = robot_params

    def forward_kinematic(self, joint_state: JointState, frame_from: int = 0, frame_to: int = 4):
        return robot_fk(joint_state, frame_from, frame_to, self.params)

    def inverse_kinematic(self, goal: CartesianGoal) -> List[JointState]:
        return robot_ik(goal, self.params)

    def jacobian(self, joint_state: JointState, frame_to: int = 4):
        return jacobian(joint_state, frame_to, self.params)
