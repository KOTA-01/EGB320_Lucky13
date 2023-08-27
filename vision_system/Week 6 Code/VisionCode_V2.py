import cv2
import numpy as np
import time

# Simulated data: Distance (in cm) and corresponding pixel counts
distances = [15, 25, 30, 35, 40, 45, 50]
pixel_counts = [125000, 53000, 40000, 26000, 21000, 16000, 13000]

# Initialize camera
cap = cv2.VideoCapture(0)

# Set camera properties
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# Initialize windows
cv2.namedWindow('Camera Feed')
cv2.namedWindow('Masked View')

color_bounds = [
    (np.array([10, 100, 100]), np.array([30, 255, 255])),   # Orange
    (np.array([40, 40, 40]), np.array([80, 255, 255])),     # Green
    (np.array([90, 50, 50]), np.array([130, 255, 255])),    # Blue
    (np.array([20, 100, 100]), np.array([40, 255, 255])),   # Yellow
    (np.array([0, 0, 0]), np.array([180, 255, 40]))         # Black
]

# Fit a polynomial regression model for pixel-to-distance mapping
coefficients = np.polyfit(pixel_counts, distances, 2)
pixel_to_distance = np.poly1d(coefficients)

# Main loop for processing video frames
while True:
    start_time = time.time()

    # Capture a frame from the camera
    ret, frame = cap.read()
    if not ret:
        break

    # Convert the frame to HSV color space
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Initialize a list to hold the masked views for each color
    masked_views = []

    for lower_bound, upper_bound in color_bounds:
        # Create a mask for the current color
        color_mask = cv2.inRange(hsv_frame, lower_bound, upper_bound)

        # Find contours in the mask
        contours, _ = cv2.findContours(color_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

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

        # Calculate the center of the largest contour
        M = cv2.moments(largest_contour)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            
            # Draw a blue circle at the center
            cv2.circle(frame, (cx, cy), 5, (255, 0, 0), -1)
            cv2.putText(frame, f'({cx},{cy})', (cx + 10, cy - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

            # Calculate the number of orange pixels in the largest contour
            num_orange_pixels = cv2.contourArea(largest_contour)

            # Estimate the distance using pixel-to-distance mapping
            estimated_distance = pixel_to_distance(num_orange_pixels)

            # Display the estimated distance on the frame
            cv2.putText(frame, f'Estimated Distance: {estimated_distance:.2f} cm', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            # Apply the mask to the original frame
            masked_frame = cv2.bitwise_and(frame, frame, mask=color_mask)
            masked_views.append(masked_frame)

            # Draw the contour on the original frame
            if largest_contour is not None:
                cv2.drawContours(frame, [largest_contour], -1, (0, 255, 0), 3)

    # Display the camera feed and the masked view
    cv2.imshow('Camera Feed', frame)
    cv2.imshow('Masked View', masked_views)

    # Wait for a key press and check if it's the 'q' key to exit the loop
    key = cv2.waitKey(1)
    if key == ord('q'):
        break

    end_time = time.time()
    #print(f"Frame processing time: {end_time - start_time:.4f} seconds")

# Release the camera and close all windows
cap.release()
cv2.destroyAllWindows()
