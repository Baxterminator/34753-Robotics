import matplotlib.pyplot as plt
import numpy as np

from robotics.kinematics import RobotModel
from robotics.trajectories import circle_path
from parameters import R, Xc, Xaxis

if __name__ == "__main__":
    # Formula for getting the torques
    # tau = J(q).T @ F
    robot = RobotModel()
    F = np.array([0, 0, -1, 0, 0, 0])

    # Compute all needed components
    [phis, goals] = circle_path(R, Xc, Xaxis)
    joint_states = [robot.inverse_kinematic(g)[0] for g in goals]
    jacobians = [robot.jacobian(j, 4) for j in joint_states]
    taus = np.array([J.T @ F for J in jacobians])
    print(taus.shape)

    fig, ax = plt.subplots(4, 1, num="Torques in the motors while moving along the circle")

    X_Ticks = [0, np.pi / 2, np.pi, 3 * np.pi / 2, 2 * np.pi]
    X_TicksLabels = [r"$\varphi=0$", r"$\varphi=\frac{\pi}{2}$", r"$\varphi=\pi$", r"$\varphi=\frac{3\pi}{2}$",
                     r"$\varphi=2\pi$"]
    for i in range(4):
        ax[i].plot(phis, taus[:, i])
        ax[i].set_ylabel(f"Joint {i + 1}")
        ax[i].set_xticks(X_Ticks)
        ax[i].set_xticklabels(["" for _ in range(len(X_Ticks))])
        ax[i].grid()
    ax[3].set_xticklabels(X_TicksLabels)

    fig.suptitle("Torques in the motors along the circle path")
    plt.tight_layout()
    plt.show()
