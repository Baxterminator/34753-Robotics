import sys
import time

import cv2
import numpy as np

from .kinematics import RobotModel
from .math import close_to
from .motors import MotorsParameters
from ._dxl import DXLWrapper, DXLFlags
from .types import JointState, RobotParameters


class RobotDriver(DXLWrapper):
    def __init__(self, interface: str, baud_rate: int, motors_param: MotorsParameters, robot_model: RobotParameters,
                 protocol=1.0, cam_id: int = 1):
        super().__init__(interface, baud_rate, motors_param, protocol)

        # Camera
        self._cam_id = cam_id
        self._cap: cv2.VideoCapture = None

        # Model
        self.model = RobotModel(robot_model)

    def __enter__(self):
        super().__enter__()
        self._cap = cv2.VideoCapture(self._cam_id)
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        super().__exit__(exc_type, exc_val, exc_tb)
        self.disengage_motors()

        # Cleanup of camera / opencv
        if self._cap is not None:
            self._cap.release()
            cv2.destroyAllWindows()

    # =========================================================================
    # Camera related functions
    # =========================================================================

    def capture_img(self) -> np.ndarray | None:
        if self._cap is None:
            return None
        ret, img = self._cap.read()
        if not ret:
            print("Error while capturing image !", file=sys.stderr)
            return None
        return img

    # =========================================================================
    # High-level functions
    # =========================================================================

    def engage_motors(self):
        if self.ping_robot():
            for i in range(1, 5):
                self.send_command(i, DXLFlags.MX_TORQUE_ENABLE, 1)

    def disengage_motors(self):
        if self.ping_robot():
            for i in range(1, 5):
                self.send_command(i, DXLFlags.MX_TORQUE_ENABLE, 0)

    def send_joints_goal(self, js: JointState) -> None:
        self.send_command(1, DXLFlags.MX_GOAL_POSITION, RobotDriver.deg_to_int_angle(js.q1))
        self.send_command(2, DXLFlags.MX_GOAL_POSITION, RobotDriver.deg_to_int_angle(js.q2))
        self.send_command(3, DXLFlags.MX_GOAL_POSITION, RobotDriver.deg_to_int_angle(js.q3))
        self.send_command(4, DXLFlags.MX_GOAL_POSITION, RobotDriver.deg_to_int_angle(js.q4))

    def get_joint_state(self) -> JointState:
        """
        Return the joint state in degrees in the range [0-300]
        """
        positions = self.get_motors_pos()
        return JointState(
            RobotDriver.int_to_deg_angle(positions[0]),
            RobotDriver.int_to_deg_angle(positions[1]),
            RobotDriver.int_to_deg_angle(positions[2]),
            RobotDriver.int_to_deg_angle(positions[3])
        )

    def get_real_joint_state(self, zero_pose: JointState) -> JointState:
        """
        Return the joint state in radians and with a value of 0 rad at the rest position
        :param zero_pose: the pose of the robot at the idle position (in degrees values)
        """
        # Convert the joint state to radians & zero-mean angles
        js = self.get_joint_state()
        return JointState(
            np.deg2rad(js.q1 - zero_pose.q1),
            np.deg2rad(js.q2 - zero_pose.q2),
            np.deg2rad(js.q3 - zero_pose.q3),
            np.deg2rad(js.q4 - zero_pose.q4)
        )

    def move_to(self, goal: JointState) -> None:
        """
        Send the joint goal for the several motors and wait for them to come at completion

        :param goal: the joint state to go to (in degrees in [0-300])
        """
        # If not on real robot, skip it
        if not self._real_robot:
            return

        # Else wait for robot to come at position
        self.send_joints_goal(goal)
        current = self.get_joint_state()
        while not current.close_to(goal, 2.0):
            print(current)
            time.sleep(0.1)
            current = self.get_joint_state()

    def move_to_real_angles(self, goal: JointState) -> None:
        """
        Send the joint goal for the several motors and wait for them to come at completion

        :param goal: the joint state to go to (in radians)
        """
        self.move_to(JointState(
            np.rad2deg(goal.q1),
            np.rad2deg(goal.q2),
            np.rad2deg(goal.q3),
            np.rad2deg(goal.q4)
        ))
