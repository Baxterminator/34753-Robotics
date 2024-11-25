import numpy as np

from parameters import R, Xc, Xaxis
from robotics.kinematics import RobotModel
from robotics.types import DEFAULT_PARAMS
from robotics.trajectories import circle_path

if __name__ == "__main__":
    robot = RobotModel(DEFAULT_PARAMS)
    [phis, goals] = circle_path(R, Xc, Xaxis, 5)

    # Compute and plot the arm configuration for each point
    for i in range(len(goals)):
        joint_state = robot.inverse_kinematic(goals[i])[0]

        J4 = robot.jacobian(joint_state, 4)
        J5 = robot.jacobian(joint_state, 5)

        print(f"Point φ = {phis[i]:.4f} rad ({np.rad2deg(phis[i]):.1f}°)")
        print(f"Joint state values: {joint_state}")
        print(f"Jacobian J4: \n {J4} \n\n Jacobian J5:\n {J5}")
        print("".ljust(60, "-"))
