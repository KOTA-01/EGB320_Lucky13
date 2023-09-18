import cv2
import numpy as np
import time

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
upper_bound = np.array([179, 255, 60])

# Camera calibration parameters (adjust as needed)
focal_length = 500  # Focal length in pixels
known_width = 10.0  # Known width of a black blob in centimeters

# Initialize variables for calculating the center point of blobs
all_blob_centroids = []

# Circular blob detection threshold
min_circularity = 0.7  # Adjust this threshold as needed


# Main loop for processing video frames
while True:
    # Capture a frame from the camera
    ret, frame = cap.read()
    if not ret:
        break

    # Measure frame processing time: Start the timer
    frame_start_time = time.time()

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

    # Initialize a list to store the centroids of blobs in the current frame
    frame_blob_centroids = []
    
    # Initialize an empty list to store estimated distances for the current frame
    estimated_distances = []

   
    # Iterate through the detected contours
    for contour in contours:
        # Calculate the area of the contour
        area = cv2.contourArea(contour)
        # Define a threshold for considering a contour as a substantial blob
        min_blob_area = 1000  # Adjust this threshold as needed
        # Calculate the perimeter of the contour
        perimeter = cv2.arcLength(contour, True)

        # Check if the perimeter is zero (or very close to zero)
        if perimeter > 0:
            # Calculate circularity using the formula: (4 * pi * area) / (perimeter^2)
            circularity = (4 * np.pi * area) / (perimeter * perimeter)

            # Define a threshold for considering a contour as a substantial circular blob
            if area >= min_blob_area and circularity >= min_circularity:
                # Find the centroid (center) of the current contour
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    centroid = (cx, cy)

                    # Append the centroid to the list of centroids for this frame
                    frame_blob_centroids.append(centroid)

                    # Increment the blob count
                    blob_count += 1

                    # Draw the contour (outline) around the blob and its centroid
                    cv2.drawContours(frame, [contour], -1, (0, 255, 0), 2)
                    cv2.circle(frame, centroid, 5, (0, 0, 255), -1)

                    # Find the bounding rectangle of the largest contour
                    x, y, w, h = cv2.boundingRect(contour)

                    # Check if the bounding rectangle touches the edge of the frame
                    edge_touching = x == 0 or y == 0 or x + w == frame.shape[1] or y + h == frame.shape[0]

                    if edge_touching:
                        # Draw a red rectangle around the largest contour
                        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
                        cv2.putText(frame, "Edge Touching", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

    # Calculate the centroid of all blobs in the current frame
    if frame_blob_centroids:
        frame_centroid_x = int(np.mean([cx for (cx, cy) in frame_blob_centroids]) - (frame.shape[1] / 2))
        frame_centroid_y = int(np.mean([cy for (cx, cy) in frame_blob_centroids]))

        # Calculate and display the estimated distance for each blob
        for contour in contours:
            area = cv2.contourArea(contour)
            if area >= min_blob_area:
                # Calculate blob width
                x, y, w, h = cv2.boundingRect(contour)
                blob_width = w

                # Calculate estimated distance
                estimated_distance = (known_width * focal_length) / blob_width
                estimated_distances.append(estimated_distance)
                # Draw the estimated distance on the frame
                cv2.putText(frame, f'Distance: {estimated_distance:.2f} cm', (x, y - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 0), 2)
        # Draw the modified centroid on the frame
        cv2.circle(frame, (frame_centroid_x + int(frame.shape[1] / 2), frame_centroid_y), 3, (255, 0, 0), -1)

        # Display the modified x, y coordinates of the average center point
        cv2.putText(frame, f'Modified Center: ({frame_centroid_x}, {frame_centroid_y})', (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 2)

    # Display the count of detected substantial black blobs
    cv2.putText(frame, f'Aisle Count: {blob_count}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 2)
    
    # Calculate the average estimated distance for the current frame
    if estimated_distances:
        average_distance = sum(estimated_distances) / len(estimated_distances)
        # Display the average distance on the frame
        cv2.putText(frame, f'Avg. Distance: {average_distance:.2f} cm', (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 0), 2)

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
