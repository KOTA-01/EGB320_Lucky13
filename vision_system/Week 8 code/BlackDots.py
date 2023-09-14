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

# Camera calibration parameters (adjust as needed)
focal_length = 500  # Focal length in pixels
known_width = 10.0  # Known width of a black blob in centimeters

# Main loop for processing video frames
while True:

    # Capture a frame from the camera
    ret, frame = cap.read()
    if not ret:
        break

    # Convert the frame to HSV color space
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Create a mask for the black color
    black_mask = cv2.inRange(hsv_frame, lower_bound, upper_bound)

    # Find contours in the mask
    contours, _ = cv2.findContours(black_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Create a masked view using bitwise operation
    masked_view = cv2.bitwise_and(frame, frame, mask=black_mask)

    # Initialize a count for substantial black blobs
    blob_count = 0

    # Iterate through the detected contours
    for contour in contours:
        # Calculate the area of the contour
        area = cv2.contourArea(contour)

        # Define a threshold for considering a contour as a substantial blob
        min_blob_area = 1000  # Adjust this threshold as needed

        if area >= min_blob_area:
            # Draw the contour (outline) around the blob
            cv2.drawContours(frame, [contour], -1, (0, 255, 0), 2)

            # Increment the blob count
            blob_count += 1

            # Calculate the distance to the blob using the known width and focal length
            distance = (known_width * focal_length) / cv2.arcLength(contour, True)  # Distance in centimeters

            # Display the distance on the frame
            cv2.putText(frame, f'Distance: {distance:.2f} cm', (contour[0][0][0], contour[0][0][1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    # Display the camera feed and the masked view
    cv2.imshow('Camera Feed', frame)
    cv2.imshow('Masked View', masked_view)

    # Display the count of detected substantial black blobs
    cv2.putText(frame, f'Aisle: {blob_count}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    cv2.imshow('Camera Feed', frame)

    # Wait for a key press and check if it's the 'q' key to exit the loop
    key = cv2.waitKey(1)
    if key == ord('q'):
        break

# Release the camera and close all windows
cap.release()
cv2.destroyAllWindows()
