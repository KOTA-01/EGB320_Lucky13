import time
import picamera2
import numpy as np
import cv2

# Simulated data: Distance (in cm) and corresponding pixel counts
distances = [15, 25, 30, 35, 40, 45, 50]
pixel_counts = [125000, 53000, 40000, 26000, 21000, 16000, 13000]


# Initialize windows
cv2.namedWindow('Camera Feed')
cv2.namedWindow('Masked View')

# Define lower and upper bounds for orange color in HSV
lower_bound = np.array([10, 100, 100])
upper_bound = np.array([30, 255, 255])

# Fit a polynomial regression model for pixel-to-distance mapping
coefficients = np.polyfit(pixel_counts, distances, 2)
pixel_to_distance = np.poly1d(coefficients)

cap = picamera2.Picamera2()
config = cap.create_video_configuration(main={"format":'XRGB8888',"size":(480,640)})
cap.configure(config)
cap.start()
while(1):
    start_time = time.time()

    frame = cap.capture_array()
    frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
    
    # Convert the frame to HSV color space
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
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
        print(num_orange_pixels)

        # Estimate the distance using pixel-to-distance mapping
        estimated_distance = pixel_to_distance(num_orange_pixels)

        # Display the estimated distance on the frame
        cv2.putText(frame, f'Estimated Distance: {estimated_distance:.2f} cm', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

    # Create a masked view using bitwise operation
    masked_view = cv2.bitwise_and(frame, frame, mask=orange_mask)

    # Draw the contour on the original frame
    if frame is not None and largest_contour is not None:
        cv2.drawContours(frame, [largest_contour], -1, (0, 255, 0), 3)
    
    
    # Display the camera feed and the masked view
    cv2.imshow('Camera Feed', frame)
    cv2.imshow('Masked View', masked_view)
    
    # Wait for a key press and check if it's the 'q' key to exit the loop
    key = cv2.waitKey(1)
    if key == ord('q'):
        break

    end_time = time.time()
    #print(f"Frame processing
    
cap.close()
cap.release()			# Release the camera object (if using opencv)
cv2.destroyAllWindows()		# Close all opencv pop-up windows
