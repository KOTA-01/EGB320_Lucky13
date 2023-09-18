import cv2
import numpy as np

# Initialize camera
cap = cv2.VideoCapture(0)  # 0 for default camera, adjust if necessary

# Set camera properties
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# Create named windows for the camera feed and masked view
cv2.namedWindow('Camera Feed')
cv2.namedWindow('Masked View')

while True:
    ret, frame = cap.read()  # Capture a frame from the camera

    # Convert the frame to HSV color space
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Define the range of orange color in HSV
    lower_bound = np.array([10, 100, 100])
    upper_bound = np.array([30, 255, 255])

    # Create a mask for the orange color
    orange_mask = cv2.inRange(hsv_frame, lower_bound, upper_bound)

    # Find contours in the mask
    contours, _ = cv2.findContours(orange_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Find the contour with the largest area (majority of orange pixels)
    largest_contour = max(contours, key=cv2.contourArea, default=None)

    # Find the bounding rectangle of the largest contour
    x, y, w, h = cv2.boundingRect(largest_contour)

    # Check if the bounding rectangle touches the edge of the frame
    edge_touching = x == 0 or y == 0 or x + w == frame.shape[1] or y + h == frame.shape[0]

    if edge_touching:
        # Draw a red rectangle around the largest contour
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
        cv2.putText(frame, "Edge Touching", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

    # Display the camera feed
    cv2.imshow('Camera Feed', frame)

    # Wait for a key press and check if it's the 'q' key to exit the loop
    key = cv2.waitKey(1)
    if key == ord('q'):
        break

# Release the camera and close all windows
cap.release()
cv2.destroyAllWindows()
