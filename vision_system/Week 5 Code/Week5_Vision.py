import cv2
import numpy as np
# import picamera2

# Initialize the webcam
cap = cv2.VideoCapture(0)  # 0 refers to the default camera
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320) # Set the width to 320 
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240) # Set the height to 240 

# Define color ranges for blue, orange, and black (TBC)
blue_lower = np.array([100, 50, 50])
blue_upper = np.array([140, 255, 255])

orange_lower = np.array([5, 100, 100])
orange_upper = np.array([20, 255, 255])

black_lower = np.array([0, 0, 0])
black_upper = np.array([180, 255, 30])

while True:
    # Capture a frame from the webcam
    ret, frame = cap.read()
    
    if ret:
        # Convert the frame to HSV color space
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Create masks for blue, orange, and black colors
        blue_mask = cv2.inRange(hsv_frame, blue_lower, blue_upper)
        orange_mask = cv2.inRange(hsv_frame, orange_lower, orange_upper)
        black_mask = cv2.inRange(hsv_frame, black_lower, black_upper)

        # Find contours for each color mask
        blue_contours, _ = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        orange_contours, _ = cv2.findContours(orange_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        black_contours, _ = cv2.findContours(black_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        
        # Display the frame
        cv2.imshow("Week 5 Vision Test", blue_mask)
        
        # Exit the loop if the 'q' key is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    else:
        print("Error capturing frame.")
        break

# Release the webcam, close OpenCV windows, and release the video writer
cap.release()
cv2.destroyAllWindows()
