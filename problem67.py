from typing import List

import numpy as np
import matplotlib.pyplot as plt

from robotics.math import force_zero
from robotics.kinematics import RobotModel, joints_velocities
from robotics.trajectories import circle_path, Quintic
from robotics.display import displayJSPlot, display_table
from parameters import R, Xc, Xaxis
from robotics.types import CartesianGoal, InterpolationNode, JointState


def compute_quintic_path(_robot: RobotModel, start: CartesianGoal, vstart: np.ndarray, end: CartesianGoal,
                         vend: np.ndarray) -> np.ndarray:
    # Manage start values
    js_start = _robot.inverse_kinematic(start)[0]
    J4_start = _robot.jacobian(js_start, 4)
    r_start = np.array([start.x - Xc.x, start.y - Xc.y, start.z - Xc.z])
    dq_start = joints_velocities(vstart, r_start, J4_start)

    # Manage end values
    js_end = _robot.inverse_kinematic(end)[0]
    J4_end = _robot.jacobian(js_end, 4)
    r_end = np.array([end.x - Xc.x, end.y - Xc.y, end.z - Xc.z])
    dq_end = joints_velocities(vend, r_end, J4_end)

    ddq = JointState(0, 0, 0, 0)

    return Quintic.solve_A_joints(
        InterpolationNode(
            0,
            js_start,
            dq_start,
            ddq
        ),
        InterpolationNode(
            2,
            js_end,
            dq_end,
            ddq
        )
    )


def print_in_console(A: np.ndarray, name: str) -> None:
    data = [[f"Joint", f"{name} t^0", f"{name} t^1", f"{name} t^2", f"{name} t^3", f"{name} t^4", f"{name} t^5"]]
    n_row, n_col = A.shape
    data.extend([[f"{A[i, j]:.4e}" if j != -1 else f"Joint {i}" for j in range(-1, n_col)] for i in range(n_row)])
    display_table(data)


def q6_get_path(robot: RobotModel, N=10):
    [phis, goals] = circle_path(R, Xc, Xaxis)

    # =====================================
    # Question 6
    # =====================================

    # Speed parameters
    start_time = 0
    v_lin = np.array([
        [0, 0, 0],
        [0, -0.027, 0],
        [0, 0, -0.027],
        [0, 0.027, 0],
        [0, 0, 0]
    ])

    # Solve coefficients
    A = compute_quintic_path(robot, goals[0], v_lin[0, :], goals[9], v_lin[1, :])
    B = compute_quintic_path(robot, goals[9], v_lin[1, :], goals[18], v_lin[2, :])
    C = compute_quintic_path(robot, goals[18], v_lin[2, :], goals[27], v_lin[3, :])
    D = compute_quintic_path(robot, goals[27], v_lin[3, :], goals[36], v_lin[4, :])

    Time = np.linspace(0, 2, N)

    def computeJointStates(js_list: List[JointState], _M: np.ndarray) -> None:
        for t in Time:
            print(f"t = {t}\n{''.ljust(24, '-')}")
            for i in range(4):
                print(
                    f"Q{i + 1}: {Quintic.compute_pos(_M[i, :], t):.3f} rad {Quintic.compute_vel(_M[i, :], t):.3f} rad/s {Quintic.compute_accel(_M[i, :], t):.3f} rad/s²")
            print()
            js_list.append(JointState(
                Quintic.compute_pos(_M[0, :], t),
                Quintic.compute_pos(_M[1, :], t),
                Quintic.compute_pos(_M[2, :], t),
                Quintic.compute_pos(_M[3, :], t)
            ))

    # Compute the joint states to display
    joints_states = []
    tout = []
    computeJointStates(joints_states, A)
    tout.extend(Time)
    computeJointStates(joints_states, B)
    tout.extend(2 + Time)
    computeJointStates(joints_states, C)
    tout.extend(4 + Time)
    computeJointStates(joints_states, D)
    tout.extend(6 + Time)
    return A, B, C, D, tout, joints_states


if __name__ == "__main__":
    robot = RobotModel()
    [A, B, C, D, _, joint_states] = q6_get_path(robot)
    [phis, goals] = circle_path(R, Xc, Xaxis)

    # =====================================
    # Question 6
    # =====================================
    # Display values in terminal
    print_in_console(A, "A")
    print_in_console(B, "B")
    print_in_console(C, "C")
    print_in_console(D, "D")


    # =====================================
    # Question 7
    # =====================================

    def plotIdealPath(_ax: plt.Axes):
        _ax.plot([p.x for p in goals], [p.y for p in goals], [p.z for p in goals], 'b--',
                 label="Desired Circle")


    displayJSPlot(
        joint_states,
        "3D Robot Arm Tracking a Circle on an interpolated path",
        robot=robot.params,
        anim=True,
        anim_hz=15,
        anim_callback=plotIdealPath
    )
