import matplotlib.pyplot as plt
import numpy as np
from typing import Tuple

from robotics.trajectories import circle_path
from robotics.types import Point3D
from robotics.kinematics import jacobian, robot_ik, RobotModel

from parameters import R, Xc, Xaxis
from problem67 import q6_get_path


def compute_path3() -> Tuple[np.ndarray, np.ndarray]:
    [phis, goals] = circle_path(R, Xc, Xaxis)
    joint_states = [robot_ik(p)[0] for p in goals]

    # On path 3
    cond = [np.linalg.cond(jacobian(js)) for js in joint_states]
    return (phis, cond)


def compute_path6() -> Tuple[np.ndarray, np.ndarray]:
    [_, _, _, _, t, joint_states] = q6_get_path(RobotModel(), 50)

    cond = [np.linalg.cond(jacobian(j)) for j in joint_states]
    return (t, cond)


if __name__ == "__main__":
    fig, ax = plt.subplots(2, 1, num="Condition number of the Jacobians")

    # Question 3 path
    [phis3, cond3] = compute_path3()
    ax[0].plot(phis3, cond3)
    ax[0].set_title("For circle path of exercise 3")
    ax[0].set_xticks([0, np.pi / 2, np.pi, 3 * np.pi / 2, 2 * np.pi])
    ax[0].set_xticklabels(
        [r"$\varphi=0$", r"$\varphi=\frac{\pi}{2}$", r"$\varphi=\pi$", r"$\varphi=\frac{3\pi}{2}$", r"$\varphi=2\pi$"])
    ax[0].grid()

    [t6, cond6] = compute_path6()
    ax[1].plot(t6, cond6)
    ax[1].set_title("For interpolated path of exercise 6")
    ax[1].grid()
    ax[1].set_xticks([0, 2, 4, 6, 8])
    ax[1].set_xticklabels([fr"$t = {t}s$" for t in [0, 2, 4, 6, 8]])
    plt.tight_layout()
    plt.show()
