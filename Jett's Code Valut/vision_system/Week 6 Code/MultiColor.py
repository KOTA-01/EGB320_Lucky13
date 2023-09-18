import cv2
import numpy as np

# Color thresholds for different colors in HSV
color_bounds = [
    (np.array([10, 100, 100]), np.array([30, 255, 255])),   # Orange
    (np.array([40, 40, 40]), np.array([80, 255, 255])),     # Green
    (np.array([90, 50, 50]), np.array([130, 255, 255])),    # Blue
    (np.array([20, 100, 100]), np.array([40, 255, 255])),    # Yellow
    (np.array([0, 0, 0]), np.array([180, 255, 40]))         # Black
]

# Initialize camera
cap = cv2.VideoCapture(0)

# Set camera properties
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# Main loop for processing video frames
while True:
    # Capture a frame from the camera
    ret, frame = cap.read()
    if not ret:
        break

    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    combined_frame = np.copy(frame)

    for lower_bound, upper_bound in color_bounds:
        # Create a mask for the current color
        color_mask = cv2.inRange(hsv_frame, lower_bound, upper_bound)

        # Apply the mask to the original frame
        masked_frame = cv2.bitwise_and(frame, frame, mask=color_mask)

        # Add the masked frame to the combined frame
        combined_frame = cv2.add(combined_frame, masked_frame)
        
    # Display the combined frame
    cv2.imshow('Combined View', combined_frame)

    # Exit loop if 'q' is pressed
    key = cv2.waitKey(1)
    if key == ord('q'):
        break

# Release the camera and close all windows
cap.release()
cv2.destroyAllWindows()
