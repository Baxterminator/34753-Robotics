import os
import time

os.environ["OPENCV_LOG_LEVEL"] = "SILENT"
import cv2
import numpy as np

from robotics.driver import RobotDriver, ValueType
from robotics.serial import get_all_ports, get_all_cam_index
from robotics.types import CartesianGoal
from robotics.vision import detect_and_smooth_curve

from parameters import ZEROS_POS, CAPTURE_POS, MOTOR_PARAMS, CAMERA_MATRIX, DISTORSION_MATRIX, ROBOT_MODEL


def choose_robot_port() -> str:
    ports = {"/dev/null": "testing without the robot", **get_all_ports()}
    ports_name = list(ports.keys())
    print("Choose port to use for the robot")
    for i in range(len(ports_name)):
        key = ports_name[i]
        print(f"- [{i}] {key} -> {ports[key]}")
    dev_id = int(input("Port to use for the robot: "))
    if 0 <= dev_id < len(ports_name):
        return ports_name[dev_id]
    raise RuntimeError(f"Invalid choice {dev_id} for robot port")


def choose_camera() -> int:
    cam = get_all_cam_index()
    print("Choose camera to use for the computations")
    for i in range(len(cam)):
        print(f"- [{i}] -> Camera #{cam[i]}")
    dev_id = int(input("Camera to use for the robot: "))
    if 0 <= dev_id < len(cam):
        return cam[dev_id]
    raise RuntimeError(f"Invalid choice {dev_id} for camera index")


# Main loop
if __name__ == "__main__":
    robot_port = choose_robot_port()
    cam_idx = choose_camera()

    with RobotDriver(robot_port, 1000000, MOTOR_PARAMS, ROBOT_MODEL, cam_id=cam_idx, zero_pose=ZEROS_POS) as driver:
        # Start the motors
        print("Initializing robot ...")
        driver.configure_robot()
        driver.engage_motors()

        print("Going to idle position")
        driver.move_to(ZEROS_POS, ValueType.DEGREES)
        driver.move_to(CAPTURE_POS, ValueType.RADIANS)

        # Move the robot towards the "capture" position
        print("\n\nGoing to capture position ...")
        driver.move_to(CAPTURE_POS, ValueType.RADIANS)
        time.sleep(1)
        img = driver.capture_img()
        if img is None:
            exit(-1)
        cv2.imshow("Captured image", img)
        cv2.waitKey(0)

        # Find the 2D points in camera frame
        capture_js = driver.get_joint_state(ValueType.RADIANS)
        camera_frame = driver.model.forward_kinematic(capture_js)
        z_coo = camera_frame[2, 3]
        camera_points = detect_and_smooth_curve(img, CAMERA_MATRIX, DISTORSION_MATRIX, z_coo)
        print("Camera points")
        print(camera_points)
        world_points: np.ndarray[float] = camera_points @ camera_frame.T
        print("World points")
        print(world_points)
        cv2.waitKey(0)

        # Follow the points
        n_points, dim = world_points.shape
        print(f"{n_points} to follow (of dim {dim})!")
        for i in range(n_points):
            print(f"Following point {i}", end=" - ")
            goal = world_points[i, :]
            possible_joint_states = driver.model.inverse_kinematic(CartesianGoal(goal[0], goal[1], 0.05, 0, 0, 1))
            N_solutions = len(possible_joint_states)
            if N_solutions == 0:
                print(f"0 solutions to get to this point")
                continue
            print(f"{N_solutions} possible solutions found")
            driver.move_to(possible_joint_states[0], ValueType.RADIANS)

        # Move back to "Idle" position
        print("Going back to idle position ...")
        driver.move_to(ZEROS_POS, ValueType.DEGREES)
