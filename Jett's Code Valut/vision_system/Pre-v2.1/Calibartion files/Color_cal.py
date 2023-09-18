import cv2
import numpy as np

# Initialize camera
cap = cv2.VideoCapture(0)  # 0 for default camera, adjust if necessary

# Set camera properties
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# Create a named window for the camera feed
cv2.namedWindow('Camera Feed')

# Initialize default mask values
lower_bound = np.array([0, 100, 100])
upper_bound = np.array([20, 255, 255])

# Create trackbars to adjust mask values
def on_red_change(val):
    lower_bound[2] = val
    upper_bound[2] = val + 20

def on_green_change(val):
    lower_bound[1] = val
    upper_bound[1] = val + 20

def on_blue_change(val):
    lower_bound[0] = val
    upper_bound[0] = val + 20

cv2.createTrackbar('Red', 'Camera Feed', lower_bound[2], 255, on_red_change)
cv2.createTrackbar('Green', 'Camera Feed', lower_bound[1], 255, on_green_change)
cv2.createTrackbar('Blue', 'Camera Feed', lower_bound[0], 255, on_blue_change)

while True:
    ret, frame = cap.read()  # Capture a frame from the camera

    # Create a mask based on the current mask values
    mask = cv2.inRange(frame, lower_bound, upper_bound)

    # Apply the mask to the original frame
    masked_frame = cv2.bitwise_and(frame, frame, mask=mask)

    # Draw rectangles and lines to indicate lower and upper bounds on each slider
    red_val = cv2.getTrackbarPos('Red', 'Camera Feed')
    green_val = cv2.getTrackbarPos('Green', 'Camera Feed')
    blue_val = cv2.getTrackbarPos('Blue', 'Camera Feed')

    cv2.rectangle(frame, (10, 30), (red_val + 10, 60), (0, 0, 255), -1)
    cv2.rectangle(frame, (10, 80), (green_val + 10, 110), (0, 255, 0), -1)
    cv2.rectangle(frame, (10, 130), (blue_val + 10, 160), (255, 0, 0), -1)

    cv2.line(frame, (red_val + 10, 45), (upper_bound[2] + 10, 45), (0, 0, 255), 2)
    cv2.line(frame, (green_val + 10, 95), (upper_bound[1] + 10, 95), (0, 255, 0), 2)
    cv2.line(frame, (blue_val + 10, 145), (upper_bound[0] + 10, 145), (255, 0, 0), 2)

    # Display the original frame, the masked frame, and the GUI window
    cv2.imshow('Camera Feed', frame)
    cv2.imshow('Masked Frame', masked_frame)

    # Wait for a key press and check if it's the 'q' key to exit the loop
    key = cv2.waitKey(1)
    if key == ord('q'):
        break

# Release the camera and close all windows
cap.release()
cv2.destroyAllWindows()
