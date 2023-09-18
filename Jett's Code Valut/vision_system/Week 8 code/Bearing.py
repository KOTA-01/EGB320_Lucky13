import cv2
import numpy as np
import time
import math

# Initialize camera
cap = cv2.VideoCapture(0)

# Set camera properties
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 308)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 410)

# Define lower and upper bounds for Black color in HSV
lower_bound = np.array([0, 0, 0])
upper_bound = np.array([179, 255, 50])

# Camera calibration parameters (adjust as needed)
focal_length = 500  # Focal length in pixels
known_width = 10.0  # Known width of a black blob in centimeters

# Initialize variables for frame processing time
total_frame_processing_time = 0
frame_count = 0

# Initialize variables for accumulating blob information
min_blob_area = 1000  # Adjust this threshold as needed

# Main loop for processing video frames
while True:

    # Measure frame processing time: Start the timer
    frame_start_time = time.time()

    ret, frame = cap.read()
    if not ret:
        break

    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    black_mask = cv2.inRange(hsv_frame, lower_bound, upper_bound)
    contours, _ = cv2.findContours(black_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Initialize variables for accumulating centers of mass
    accumulated_center_x = 0
    accumulated_center_y = 0
    num_qualifying_blobs = 0

    for contour in contours:
        area = cv2.contourArea(contour)

        if area >= min_blob_area:
            # Calculate the center of mass (centroid) of the blob
            M = cv2.moments(contour)
            if M['m00'] > 0:
                center_x = int(M['m10'] / M['m00'])
                center_y = int(M['m01'] / M['m00'])

                # Display the center of mass on the frame
                cv2.circle(frame, (center_x, center_y), 5, (0, 255, 0), -1)

                # Accumulate the centers of mass
                accumulated_center_x += center_x
                accumulated_center_y += center_y
                num_qualifying_blobs += 1

    # Calculate the overall center point based on the centers of mass
    if num_qualifying_blobs > 0:
        overall_center_x = accumulated_center_x / num_qualifying_blobs
        overall_center_y = accumulated_center_y / num_qualifying_blobs

        # Display the overall center point on the frame
        cv2.putText(frame, f'Center (X, Y): ({overall_center_x:.2f}, {overall_center_y:.2f}) pixels', (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

    # Measure frame processing time: Stop the timer
    frame_end_time = time.time()

    # Calculate and accumulate the frame processing time
    frame_processing_time = frame_end_time - frame_start_time
    total_frame_processing_time += frame_processing_time
    frame_count += 1

    cv2.imshow('Camera Feed', frame)

    key = cv2.waitKey(1)
    if key == ord('q'):
        break

# Calculate and print the average time spent per frame
average_frame_processing_time = total_frame_processing_time / frame_count
print(f'Average Time Per Frame: {average_frame_processing_time:.4f} sec')

cap.release()
cv2.destroyAllWindows()
