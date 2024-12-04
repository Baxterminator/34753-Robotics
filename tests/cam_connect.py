import os

os.environ["OPENCV_LOG_LEVEL"] = "SILENT"
import cv2

if __name__ == "__main__":
    cap = cv2.VideoCapture(2)
    if not cap.isOpened():
        print("Could not connect to camera")
        exit(-1)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)  # Fixed the property name
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1080)  # Fixed the property name
    cap.set(cv2.CAP_PROP_FPS, 30)  # Fixed the property name

    ret, img = cap.read()
    while True:
        cv2.imshow("Capture", img)
        ret, img = cap.read()
        if cv2.waitKey(1) == ord('q'):
            break
    cap.release()
