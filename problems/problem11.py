import numpy as np
import time
import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../')))

import cv2
from scipy.interpolate import splprep, splev

from robot.kinematics import force_zero, robot_ik, robot_fk, jacobians
from robot.types import JointState, CartesianGoal, DEFAULT_PARAMS, T45, CAMERA_MATRIX, DIST_COEFF
import dynamixel_sdk as dxl

# parameters for the servos
ADDR_MX_TORQUE_ENABLE = 24
ADDR_MX_CW_COMPLIANCE_MARGIN = 26
ADDR_MX_CCW_COMPLIANCE_MARGIN = 27
ADDR_MX_CW_COMPLIANCE_SLOPE = 28
ADDR_MX_CCW_COMPLIANCE_SLOPE = 29
ADDR_MX_GOAL_POSITION = 30
ADDR_MX_MOVING_SPEED = 32
ADDR_MX_PRESENT_POSITION = 36
ADDR_MX_PUNCH = 48
PROTOCOL_VERSION = 1.0
DXL_IDS = [1, 2, 3, 4]
DEVICENAME = "COM11"
BAUDRATE = 1000000
TORQUE_ENABLE = 1
TORQUE_DISABLE = 0
portHandler = dxl.PortHandler(DEVICENAME)
packetHandler = dxl.PacketHandler(PROTOCOL_VERSION)
portHandler.openPort()
portHandler.setBaudRate(BAUDRATE)

def rotate2real(angles: list = [0,0,0,0]):
    return [angles[0] + 150, angles[1] + 150, angles[2] + 150, angles[3] + 240]

def picture_pose():
    # set the robot to picture pose
    pose_angles = [0,-45,-22.5,-(90+22.5)]
    angles = rotate2real(pose_angles)
    # scale to servo angles (0-300) <-->(0-1023)
    dxl_angles = [int(angle / 300 * 1023) for angle in angles]
    # give time to reach the position
    #time.sleep(10)
    for DXL_ID in DXL_IDS:
        packetHandler.write1ByteTxRx(
            portHandler, DXL_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE
        )
        packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_MX_MOVING_SPEED, 90)
        packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_MX_GOAL_POSITION, dxl_angles[DXL_ID-1])
        
    time.sleep(8)
        
    # convert pose angles in radiants
    pose_angles = [angle*np.pi/180 for angle in pose_angles]  
    print(pose_angles)
    return(JointState(pose_angles[0],
                      pose_angles[1],
                      pose_angles[2],
                      pose_angles[3]))

###
# computer vision functions for recognizing the line 
###

# Opens a video capture device with a resolution of 800x600
# at 30 FPS.
def open_camera(cam_id=1):
    cap = cv2.VideoCapture(cam_id,cv2.CAP_DSHOW)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)  # Fixed the property name
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1080)   # Fixed the property name
    cap.set(cv2.CAP_PROP_FPS, 30)            # Fixed the property name
    if not cap.isOpened():
        print("Error: Unable to open the camera.")
        sys.exit(1)
    return cap

# Closes all OpenCV windows and releases video capture device
# before exit.
def cleanup(device):
    device.release()
    cv2.destroyAllWindows()
    
    
def detect_and_smooth_curve(image_path, z_cam,camera_matrix, dist_coeffs):
    # Load the image
    img = cv2.imread(image_path)
    if img is None:
        raise FileNotFoundError(f"Image not found at {image_path}. Please check the file path.")

    # Convert the image to grayscale
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Apply GaussianBlur to reduce noise
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)

    # Adjusted thresholding to isolate the black curve
    _, thresh = cv2.threshold(blurred, 100, 255, cv2.THRESH_BINARY_INV)

    # Find contours
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        print("No contours found.")
        return []

    # Find the largest contour (assuming it corresponds to the curve)
    curve_contour = max(contours, key=cv2.contourArea)

    # Approximate the curve with a series of points
    epsilon = 0.0005 * cv2.arcLength(curve_contour, True)  # Adjust epsilon to control approximation precision
    approx_curve = cv2.approxPolyDP(curve_contour, epsilon, True)

    # Filter redundant points (remove points that are too close to each other)
    filtered_points = [approx_curve[0]]
    for point in approx_curve[1:]:
        if np.linalg.norm(filtered_points[-1][0] - point[0]) > 50:  # Minimum distance threshold (5 pixels)
            filtered_points.append(point)

    # Extract x, y coordinates
    filtered_points = np.array([point[0] for point in filtered_points], dtype=np.float32)

    # Smooth the curve using a spline
    x, y = filtered_points[:, 0], filtered_points[:, 1]
    tck, _ = splprep([x, y], s=1)  # Smoothness factor (s)
    u_fine = np.linspace(0, 1, 100)  # Number of points for the smooth curve
    x_smooth, y_smooth = splev(u_fine, tck)
    smoothed_points = np.vstack((x_smooth, y_smooth)).T
    
    # Undistort image points
    undistorted_points = cv2.undistortPoints(filtered_points, camera_matrix, dist_coeffs)
    mm_points = undistorted_points * (z_cam + 0.045)
    camera_points = np.hstack((mm_points.squeeze(), np.ones((len(mm_points), 1))*(z_cam-0.010)))  # Add Z = z_cam

    # # Transform points to world coordinates
    # rotation_matrix, _ = cv2.Rodrigues(rotation_vector)  # Convert rotation vector to matrix
    # world_points = (rotation_matrix @ undistorted_points.T).T + translation_vector.T

    # Draw the smoothed curve on the image for visualization
    output_img = img.copy()
    for i in range(len(smoothed_points) - 1):
        pt1 = tuple(smoothed_points[i].astype(int))
        pt2 = tuple(smoothed_points[i + 1].astype(int))
        cv2.line(output_img, pt1, pt2, (0, 255, 0), 2)
        
    for i in range (len(filtered_points)):
        #print((int(filtered_points[i,0]),int(filtered_points[i,1])))
        cv2.circle(output_img, (int(filtered_points[i,0]),int(filtered_points[i,1])), 5, (255,0,0), 10)
    

    # Display the result
    cv2.imshow("Smoothed Curve", output_img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    # Return the real-world coordinates of the curve
    return camera_points
    
    
if __name__ == "__main__":
    ###
    # DEFINE THE HEIGHT OF CAMERA
    ###
    js_cam = picture_pose()
    T04 = robot_fk(js_cam)
    T05= T04 @ T45
    
    z_cam = T05[0:3,3][2]

    #################################
    
    ###
    # TAKE THE PICTURE
    ###
    # Camera ID to read video from (numbered from 0)
    camera_id = 1
    dev = open_camera(camera_id)  # Open the camera as a video capture device
    
    # path to save image
    output = os.getcwd()
    output_path = os.path.join(output,"a8_with_line.jpg")  
    #print(output_path)

    try:
        while True:
            ret, frame = dev.read()
            if not ret:
                print("error in capturing frame")
                break
            
            cv2.imshow("video", frame)  # Display the image in a window named "video"
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord('s'):  # Save the frame if press 's'
                cv2.imwrite(output_path, frame)
                print(f"Frame saved as {output_path}")
                break
            elif key == ord('q'):  # Exit the loop if prss 'q'
                break

    finally:
        # Always cleanup resources, even if an error occurs
        cleanup(dev)
    
    #####################################################################
    
    ###
    # recognize line and get the points coordinates
    ###
    # Detect and smooth the curve, get real-world camera frame coordinates
    curve_points = detect_and_smooth_curve(output_path, z_cam, CAMERA_MATRIX, DIST_COEFF)

    # Print the real-world coordinates
    for i, point in enumerate(curve_points):
        print(f"Real-world coordinates of point {i + 1}: {point} m")
    
    #####################################################################

    ###
    # get the real world coordinate of each point w.r.t image frame
    ###
    #list of global ref coordinates points
    points = [] 
    # rotate from image frame to camera frame

    for point_l in curve_points:
        
        point = np.append(np.array(point_l),1)
        point = point.reshape(-1,1)
        
        # compute the global point
        T56 = np.array([
                            [0, 0, 1, 0],
                            [0, -1, 0, 0],
                            [1, 0, 0, 0],
                            [0, 0, 0, 1]
                        ])  # camera frame with respect to robot camera frame
        T06 = T05 @ T56
        print(point)
        p = T06 @ point
        
        #print(f"point{p}")
        
        points.append(CartesianGoal(x=p[0,0],y=p[1,0],z=p[2,0],rx=0,ry=0,rz=-1))
        
    ##################################################################
        
    ###
    # compute inverse kinematics and move the robot
    ###
    for i in range(len(points)):
        joint_states = robot_ik(points[i])    
        #print(joint_states)
        for i in range(len(joint_states)):
            angles = rotate2real([np.degrees(joint_states[i].q1), np.degrees(joint_states[i].q2), np.degrees(joint_states[i].q3), np.degrees(joint_states[i].q4)])

            # Verifica che tutti gli angoli siano compresi tra 0 e 300
            are_valid = all(0 <= angle <= 300 for angle in angles)
            if are_valid and angles[1]>29:
                js = joint_states[i]
                break
            
        # # compute the jacobian for the first solution:
        # T = []
        # for idx in range(1, 5):
        #     T.append(robot_fk(js, 0, idx)) 
        # J = jacobians(T)
        angles = rotate2real([np.degrees(js.q1),
                      np.degrees(js.q2),
                      np.degrees(js.q3),
                      np.degrees(js.q4)])
        
        # move the joints
        for DXL_ID in DXL_IDS:
            # Convert angle to position value for the AX-12A
            # Target angles for servos (0° to 300°)
            dxl_goal_position = int((angles[DXL_ID-1] / 300.0) * 1023)
            
            # Convert speed to speed value for the AX-12A
            # dxl_speed = int((speeds[DXL_ID-1] / 354.0) * 1023)
            
            packetHandler.write1ByteTxRx(
                portHandler, DXL_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE
            )
            packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_MX_CW_COMPLIANCE_MARGIN, 0)
            packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_MX_CCW_COMPLIANCE_MARGIN, 0)
            packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_MX_CW_COMPLIANCE_SLOPE, 32)
            packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_MX_CCW_COMPLIANCE_SLOPE, 32)
            packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_MX_MOVING_SPEED, 90)
            packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_MX_GOAL_POSITION, dxl_goal_position)
            
        time.sleep(0.5)
        
    picture_pose()
    



        
    

    
