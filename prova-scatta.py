#!/usr/bin/python

import sys
import cv2
import os
import numpy

##
# Opens a video capture device with a resolution of 800x600
# at 30 FPS.
##
def open_camera(cam_id=1):
    cap = cv2.VideoCapture(cam_id)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)  # Fixed the property name
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1080)   # Fixed the property name
    cap.set(cv2.CAP_PROP_FPS, 30)            # Fixed the property name
    if not cap.isOpened():
        print("Error: Unable to open the camera.")
        sys.exit(1)
    return cap

##
# Gets a frame from an open video device, or returns None
# if the capture could not be made.
##
def get_frame(device):
    ret, img = device.read()
    if not ret:  # failed to capture
        sys.stderr.write("Error capturing from video device.\n")
        return None
    return img

##
# Closes all OpenCV windows and releases video capture device
# before exit.
##
def cleanup(device):
    device.release()
    cv2.destroyAllWindows()

########### Main Program ###########

if __name__ == "__main__":
    # Camera ID to read video from (numbered from 0)
    camera_id = 1
    dev = open_camera(camera_id)  # Open the camera as a video capture device
    captured_image = None  # To store the last frame
    output = os.getcwd()
    output_path = os.path.join(output,"a8_with_line.jpg")  
    print(output_path)

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
