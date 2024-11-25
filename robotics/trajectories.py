import numpy as np
from typing import Tuple, List
from robotics.math import gram_schmidt, powers_array
from robotics.types import Point3D, InterpolationNode, JointState
from robotics.kinematics import CartesianGoal


def circle_path(R: float, center: Point3D, rot_axis: Point3D, N: int = 37) -> Tuple[np.ndarray, List[CartesianGoal]]:
    """
    Compute a joint path for circle path in cartesian space from its geometrical properties.

    :param N: the number of point to generate on the circle
    :param R: the radius of the circle
    :param center: the center of the circle
    :param rot_axis: the axis around which the circle revolve around
    :return: a list of joint state to move around the circle
    """

    # Generate N equidistant points on the circle in the plane
    phis = np.linspace(0, 2 * np.pi, N)
    M = gram_schmidt(rot_axis.to_arr())
    path = [(R * np.cos(phi), R * np.sin(phi)) for phi in phis]
    return (phis, [CartesianGoal(x=center.x + M[0, 0] * p[0] + M[0, 1] * p[1],
                                 y=center.y + M[1, 0] * p[0] + M[1, 1] * p[1],
                                 z=center.z + M[2, 0] * p[0] + M[2, 1] * p[1],
                                 rz=0)
                   for p in path])


class Quintic:
    DEFAULT_N_POINTS = 10

    @staticmethod
    def _pos_coefficients(t: float) -> np.ndarray:
        return powers_array(t, 5)

    @staticmethod
    def compute_pos(A: np.ndarray, t: float) -> float:
        return np.dot(A, Quintic._pos_coefficients(t))

    @staticmethod
    def _vel_coefficients(t: float) -> np.ndarray:
        time_p = powers_array(t, 4)
        return np.array([
            0,
            time_p[0],
            2 * time_p[1],
            3 * time_p[2],
            4 * time_p[3],
            5 * time_p[4]
        ])

    @staticmethod
    def compute_vel(A: np.ndarray, t: float) -> float:
        return np.dot(A, Quintic._vel_coefficients(t))

    @staticmethod
    def _accel_coefficients(t: float) -> np.ndarray:
        time_p = powers_array(t, 3)
        return np.array([
            0,
            0,
            2 * time_p[0],
            6 * time_p[1],
            12 * time_p[2],
            20 * time_p[3]
        ])

    @staticmethod
    def compute_accel(A: np.ndarray, t: float) -> float:
        return np.dot(A, Quintic._accel_coefficients(t))

    @staticmethod
    def solve_A_numeric(start: InterpolationNode[float], end: InterpolationNode[float]):
        # Coefficient and constant matrices for linear system
        A = np.array([
            Quintic._pos_coefficients(start.t),
            Quintic._vel_coefficients(start.t),
            Quintic._accel_coefficients(start.t),
            Quintic._pos_coefficients(end.t),
            Quintic._vel_coefficients(end.t),
            Quintic._accel_coefficients(end.t),
        ])
        b = np.array([start.q, start.dq, start.ddq, end.q, end.dq, end.ddq])

        # solve linear system
        X = np.linalg.solve(A, b)
        return np.reshape(X, (1, 6))

    @staticmethod
    def solve_A_joints(start: InterpolationNode[JointState], end: InterpolationNode[JointState]):
        A = np.zeros((4, 6))
        for idx in range(1, 5):
            A[idx - 1, :] = Quintic.solve_A_numeric(
                InterpolationNode(
                    start.t,
                    start.q[idx],
                    start.dq[idx],
                    start.ddq[idx],
                ),
                InterpolationNode(
                    end.t,
                    end.q[idx],
                    end.dq[idx],
                    end.ddq[idx],
                ),
            )
        return A
