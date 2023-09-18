import cv2
import numpy as np

# Initialize the camera (adjust the camera index as needed)
cap = cv2.VideoCapture(0)

while True:
    # Capture a frame from the camera
    ret, frame = cap.read()

    if not ret:
        break

    # Convert the frame to grayscale for edge detection
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Apply Gaussian blur to reduce noise
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)

    # Define the lower and upper bounds for blue color
    lower_blue = np.array([100, 50, 50])
    upper_blue = np.array([130, 255, 255])

    # Create a mask to isolate blue regions
    mask_blue = cv2.inRange(frame, lower_blue, upper_blue)

    # Perform edge detection using Canny
    edges = cv2.Canny(blurred, 50, 150)

    # Find lines in the edges using Hough Line Transform
    lines = cv2.HoughLinesP(edges, 1, np.pi / 180, 100, minLineLength=100, maxLineGap=10)

    if lines is not None:
        # Initialize variables to store non-blue centerline points
        non_blue_points = []

        for line in lines:
            x1, y1, x2, y2 = line[0]

            # Calculate the angle of the line
            angle = np.arctan2(y2 - y1, x2 - x1) * 180 / np.pi

            # Filter out lines that are within a certain angle range (e.g., exclude near-vertical lines)
            if 10 < abs(angle) < 80:
                # Calculate the midpoint of the line
                mid_x = (x1 + x2) // 2
                mid_y = (y1 + y2) // 2

                # Check if the midpoint is not within the blue region
                if mask_blue[mid_y, mid_x] != 255:
                    non_blue_points.append((mid_x, mid_y))

        # Calculate the centerline based on the identified non-blue points
        if non_blue_points:
            center_x = sum([x for x, _ in non_blue_points]) // len(non_blue_points)
            center_y = max([y for _, y in non_blue_points])
        else:
            center_x = frame.shape[1] // 2
            center_y = frame.shape[0] // 2

        # Draw the centerline
        cv2.line(frame, (center_x, center_y), (center_x, frame.shape[0]), (0, 255, 0), 2)

    # Show the masked view of the blue shelves and grey carpet
    masked_view = cv2.bitwise_and(frame, frame, mask=mask_blue)
    cv2.imshow('Masked View', masked_view)

    # Display the frame with the detected centerline
    cv2.imshow('Centerline Detection', frame)

    # Exit the loop when 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the camera and close all OpenCV windows
cap.release()
cv2.destroyAllWindows()
