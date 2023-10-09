import cv2
import numpy as np

# Known dimensions of the orange cube (in centimeters)
cube_width = 5.0
cube_height = 5.0
cube_depth = 5.0

# Camera calibration parameters (obtained previously)
camera_matrix = np.array([[fx, 0, cx],
                           [0, fy, cy],
                           [0, 0, 1]])
dist_coeffs = np.array([k1, k2, p1, p2, k3])

# Open the camera
cap = cv2.VideoCapture(0)  # Use 0 for the default camera, you can change it if needed

while True:
    # Read a frame from the camera
    ret, frame = cap.read()
    if not ret:
        break

    # Detect the orange cube in the frame using color segmentation
    lower_range = np.array([10, 100, 20])
    upper_range = np.array([30, 255, 255])
    mask = cv2.inRange(frame, lower_range, upper_range)
    masked_frame = cv2.bitwise_and(frame, frame, mask=mask)

    # Find contours in the mask to locate the orange cube
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Collect image points (2D) representing the cube's corners
    image_points = []

    for contour in contours:
        if cv2.contourArea(contour) > 5000:
            epsilon = 0.1 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)
            if len(approx) == 8:
                for point in approx:
                    x, y = point[0]
                    image_points.append([x, y])

    # If you have successfully detected the orange cube and collected image points
    if len(image_points) == 8:
        # Calculate the 3D object points corresponding to the cube's corners
        object_points = np.array([
            [0, 0, 0],
            [cube_width, 0, 0],
            [cube_width, cube_height, 0],
            [0, cube_height, 0],
            [0, 0, -cube_depth],
            [cube_width, 0, -cube_depth],
            [cube_width, cube_height, -cube_depth],
            [0, cube_height, -cube_depth]
        ])

        # Use solvePnP to estimate the pose (rotation and translation)
        _, rvecs, tvecs, inliers = cv2.solvePnPRansac(object_points, np.array(image_points),
                                                     camera_matrix, dist_coeffs)

        # Calculate the distance from the camera to the cube (Z-coordinate in millimeters)
        distance_mm = tvecs[2][0]

        # Print the estimated distance
        print(f"Estimated Distance: {distance_mm / 10} cm")  # Convert to centimeters

    # ... (continue with the existing code for object detection and visualization)

    # Exit the loop if the 'q' key is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the camera and close all OpenCV windows
cap.release()
cv2.destroyAllWindows()
