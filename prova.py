import cv2
import numpy as np
from scipy.interpolate import splprep, splev
import os

def detect_and_smooth_curve(image_path, camera_matrix, dist_coeffs):
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
    camera_points = np.hstack((undistorted_points.squeeze(), np.zeros((len(undistorted_points), 1))))  # Add Z = 0

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


# Main Function
if __name__ == "__main__":
    # Camera parameters (replace with actual values)
    camera_matrix = np.array([[1068.0, 0, 506.8],
                              [0, 1083.0, 371.8],
                              [0, 0, 1]])  # Intrinsic parameters

    dist_coeffs = np.array([0.1158, 0.0412, 0, 0, 0])  # Distortion coefficients
    # rotation_vector = np.array([[0.1335], [0.0274], [0.0131]])  # Rotation vector
    # translation_vector = np.array([[-3.2742], [-3.3057], [18.4347]])  # Translation vector

    # Path to your image
    image = os.getcwd()
    image_path = os.path.join(image,"a8_with_line.jpg")  # Replace with your actual image path

    # Detect and smooth the curve, get real-world camera frame coordinates
    curve_points = detect_and_smooth_curve(image_path, camera_matrix, dist_coeffs)

    # Print the real-world coordinates
    for i, point in enumerate(curve_points):
        print(f"Real-world coordinates of point {i + 1}: {point}")