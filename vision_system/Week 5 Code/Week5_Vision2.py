import cv2
import numpy as np

# Simulated data: Distance (in cm) and corresponding pixel counts
distances = [50, 100, 150, 200, 250]
pixel_counts = [150, 300, 500, 1000, 2000]

# Fit a polynomial regression model
coefficients = np.polyfit(pixel_counts, distances, 2)  # Second-degree polynomial
pixel_to_distance = np.poly1d(coefficients)

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

    # Calculate the center of the largest contour
    M = cv2.moments(largest_contour)
    if M["m00"] != 0:
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        
        # Check if the contour is touching the edge of the screen
        edge_error = cx <= 0 or cx >= frame.shape[1] - 1 or cy <= 0 or cy >= frame.shape[0] - 1

        if not edge_error:
            # Draw a blue circle at the center
            cv2.circle(frame, (cx, cy), 5, (255, 0, 0), -1)
            cv2.putText(frame, f'({cx},{cy})', (cx + 10, cy - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

            # Calculate the number of orange pixels in the largest contour
            num_orange_pixels = cv2.contourArea(largest_contour)

            # Estimate the distance using pixel-to-distance mapping
            estimated_distance = pixel_to_distance(num_orange_pixels)

            # Display the estimated distance on the frame
            cv2.putText(frame, f'Estimated Distance: {estimated_distance:.2f} cm', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        else:
            # Display the edge error warning
            cv2.putText(frame, "Edge Error!", (10, frame.shape[0] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

    # Create a masked view using bitwise operation
    masked_view = cv2.bitwise_and(frame, frame, mask=orange_mask)

    # Draw the contour on the original frame
    cv2.drawContours(frame, [largest_contour], -1, (0, 255, 0), 3)

    # Draw edges of the output view
    cv2.rectangle(frame, (0, 0), (frame.shape[1] - 1, frame.shape[0] - 1), (0, 255, 0), 2)

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
