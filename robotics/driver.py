import sys
import time
from enum import Enum

import cv2
import numpy as np

from .kinematics import RobotModel
from .math import normalize_angle
from .motors import MotorsParameters
from ._dxl import DXLWrapper, DXLFlags
from .types import JointState, RobotParameters


class ValueType(Enum):
    RADIANS = 0
    DEGREES = 1
    INT = 2


class RobotDriver(DXLWrapper):
    def __init__(self, interface: str, baud_rate: int, motors_param: MotorsParameters, robot_model: RobotParameters,
                 protocol=1.0, cam_id: int = 1, zero_pose=JointState(0, 0, 0, 0)):
        super().__init__(interface, baud_rate, motors_param, protocol)

        # Camera
        self._cam_id = cam_id
        self._cap: cv2.VideoCapture = None

        # Model
        self.model = RobotModel(robot_model)
        self.zero_pose = zero_pose

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
        if self._cap is None or not self._cap.isOpened():
            print("Error while connecting to the camera !", file=sys.stderr)
            return None
        ret, img = self._cap.read()
        if not ret:
            print("Error while capturing image !", file=sys.stderr)
            return None
        return img

    # =========================================================================
    # Math related function
    # =========================================================================
    def real_to_int_angles(self, val: JointState) -> JointState:
        def convert(v: float, zero: float) -> float:
            return RobotDriver.deg_to_int_angle(np.rad2deg(normalize_angle(v)) + zero)

        return JointState(
            convert(val.q1, self.zero_pose.q1),
            convert(val.q2, self.zero_pose.q2),
            convert(val.q3, self.zero_pose.q3),
            convert(val.q4, self.zero_pose.q4)
        )

    def int_to_real_angles(self, val: JointState) -> JointState:
        def convert(v: float, zero: float) -> float:
            return normalize_angle(np.deg2rad((RobotDriver.int_to_deg_angle(int(v)) - zero)))

        return JointState(
            convert(val.q1, self.zero_pose.q1),
            convert(val.q2, self.zero_pose.q2),
            convert(val.q3, self.zero_pose.q3),
            convert(val.q4, self.zero_pose.q4)
        )

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

    def send_joints_goal(self, js: JointState, t=ValueType.RADIANS) -> JointState:
        print(f"Sending goal {js}")
        match t:
            case ValueType.RADIANS:
                return self.send_joints_goal(self.real_to_int_angles(js), ValueType.INT)
            case ValueType.DEGREES:
                return self.send_joints_goal(JointState(self.deg_to_int_angle(js.q1), self.deg_to_int_angle(js.q2),
                                                        self.deg_to_int_angle(js.q3), self.deg_to_int_angle(js.q4)),
                                             ValueType.INT)
            case ValueType.INT:
                self.send_command(1, DXLFlags.MX_GOAL_POSITION, int(js.q1))
                self.send_command(2, DXLFlags.MX_GOAL_POSITION, int(js.q2))
                self.send_command(3, DXLFlags.MX_GOAL_POSITION, int(js.q3))
                self.send_command(4, DXLFlags.MX_GOAL_POSITION, int(js.q4))
                return js

    def get_joint_state(self, t=ValueType.RADIANS) -> JointState:
        """
        Return the joint state in degrees in the range [0-300]
        """
        positions = self.get_motors_pos()
        match t:
            case ValueType.RADIANS:
                return self.int_to_real_angles(JointState(
                    positions[0],
                    positions[1],
                    positions[2],
                    positions[3]
                ))
            case ValueType.DEGREES:
                return JointState(
                    RobotDriver.int_to_deg_angle(positions[0]),
                    RobotDriver.int_to_deg_angle(positions[1]),
                    RobotDriver.int_to_deg_angle(positions[2]),
                    RobotDriver.int_to_deg_angle(positions[3])
                )
            case ValueType.INT:
                return JointState(
                    positions[0],
                    positions[1],
                    positions[2],
                    positions[3]
                )

    def move_to(self, goal: JointState, t=ValueType.RADIANS) -> None:
        """
        Send the joint goal for the several motors and wait for them to come at completion

        :param t: the type of the values in the joint state
        :param goal: the joint state to go to
        """
        # If not on real robot, skip it
        if not self._real_robot:
            return

        # Else wait for robot to come at position
        goal_int = self.send_joints_goal(goal, t)
        current = self.get_joint_state(ValueType.INT)
        print(f"-------\nGoal   : {goal_int}\nCurrent: {current}\nDelta  : {current - goal_int}")
        while not current.close_to(goal_int, 10):
            time.sleep(0.1)
            current = self.get_joint_state(ValueType.INT)
            print(f"-------\nGoal   : {goal_int}\nCurrent: {current}\nDelta  : {current - goal_int}")
