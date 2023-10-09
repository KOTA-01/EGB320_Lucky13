import cv2
import numpy as np

# Initialize camera
cap = cv2.VideoCapture(0)

# # Set camera properties
# cap.set(cv2.CAP_PROP_FRAME_WIDTH, 308)
# cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 410)

# Initialize windows
cv2.namedWindow('Camera Feed')
cv2.namedWindow('Masked View')

# Simulated data: Distance (in cm) and corresponding pixel counts
distances = [15, 20, 25, 30]
pixel_width = [441, 335, 271, 232]

# Define lower and upper bounds for Red color in HSV
lower_bound = np.array([0, 100, 100])   # Lower bound for red color
upper_bound = np.array([10, 255, 255])  # Upper bound for red color

# Fit a polynomial regression model for pixel-to-distance mapping
coefficients = np.polyfit(pixel_width, distances, 2)
pixel_to_distance = np.poly1d(coefficients)

# Main loop for processing video frames
while True:
    # Capture a frame from the camera
    ret, frame = cap.read()
    if not ret:
        break

    # Convert the frame to HSV color space
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Create a mask for the red color
    red_mask = cv2.inRange(hsv_frame, lower_bound, upper_bound)

    # Find contours in the mask
    contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Create a masked view using bitwise operation
    masked_view = cv2.bitwise_and(frame, frame, mask=red_mask)

    # Initialize variables for calculating the center point of objects
    all_object_centroids = []

    # Initialize an empty list to store estimated distances for the current frame
    estimated_distances = []

    # Iterate through the detected contours
    for contour in contours:
        # Calculate the area of the contour
        area = cv2.contourArea(contour)
        # Define a threshold for considering a contour as an object
        min_object_area = 1000  # Adjust this threshold as needed

        if area >= min_object_area:
            # Find the centroid (center) of the current contour
            M = cv2.moments(contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                centroid = (cx, cy)

                # Append the centroid to the list of centroids for this frame
                all_object_centroids.append(centroid)

                # Draw the contour (outline) around the object and its centroid
                cv2.drawContours(frame, [contour], -1, (0, 255, 0), 2)
                cv2.circle(frame, centroid, 5, (0, 0, 255), -1)

                # Find the bounding rectangle of the contour
                x, y, w, h = cv2.boundingRect(contour)

                # Calculate the width of the object
                object_width = w

                # Estimate the distance using pixel-to-distance mapping
                estimated_distance = pixel_to_distance(object_width)
                estimated_distances.append(estimated_distance)

                # Display the estimated distance on the frame
                cv2.putText(frame, f'Distance: {estimated_distance:.2f} cm', (x, y - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                            (255, 0, 0), 2)

    # Calculate the average estimated distance for the current frame
    if estimated_distances:
        average_distance = sum(estimated_distances) / len(estimated_distances)
        # Display the average distance on the frame
        cv2.putText(frame, f'Avg. Distance: {average_distance:.2f} cm', (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                    (255, 0, 0), 2)

    # Display the camera feed and the masked view
    cv2.imshow('Camera Feed', frame)
    cv2.imshow('Masked View', masked_view)

    # Wait for a key press and check if it's the 'q' key to exit the loop
    key = cv2.waitKey(1)
    if key == ord('q'):
        break

# Release the camera and close all windows
cap.release()
cv2.destroyAllWindows()
