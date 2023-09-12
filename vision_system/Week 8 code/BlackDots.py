import cv2
import numpy as np

# Initialize camera
cap = cv2.VideoCapture(0)

# Set camera properties
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 308)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 410)

# Initialize windows
cv2.namedWindow('Camera Feed')
cv2.namedWindow('Masked View')

# Define lower and upper bounds for Black color in HSV
lower_bound = np.array([0, 0, 0])
upper_bound = np.array([179, 255, 50])


# Main loop for processing video frames
while True:

    # Capture a frame from the camera
    ret, frame = cap.read()
    if not ret:
        break

    # Convert the frame to HSV color space
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Create a mask for the orange color
    black_mask = cv2.inRange(hsv_frame, lower_bound, upper_bound)

    # Find contours in the mask
    contours, _ = cv2.findContours(black_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Create a masked view using bitwise operation
    masked_view = cv2.bitwise_and(frame, frame, mask=black_mask)

    # Display the camera feed and the masked view
    cv2.imshow('Camera Feed', frame)
    cv2.imshow('Masked View', masked_view)

    # Wait for a key press and check if it's the 'q' key to exit the loop
    key = cv2.waitKey(1)
    if key == ord('q'):
        break

    end_time = time.time()
    #print(f"Frame processing time: {end_time - start_time:.4f} seconds")

# Release the camera and close all windows
cap.release()
cv2.destroyAllWindows()
