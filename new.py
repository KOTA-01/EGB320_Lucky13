import cv2
import numpy as np

# Initialize the webcam
cap = cv2.VideoCapture(0)  # 0 refers to the default camera

# Define color ranges for blue, orange, and black
blue_lower = np.array([100, 50, 50])
blue_upper = np.array([140, 255, 255])

orange_lower = np.array([5, 100, 100])
orange_upper = np.array([20, 255, 255])

black_lower = np.array([0, 0, 0])
black_upper = np.array([180, 255, 30])

# Define a video writer
output_writer = cv2.VideoWriter('circle_detection_output.avi', cv2.VideoWriter_fourcc(*'XVID'), 20.0, (640, 480))

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

        # Detect circles within the blue mask
        blue_circles = cv2.HoughCircles(blue_mask, cv2.HOUGH_GRADIENT, dp=1, minDist=30,
                                        param1=50, param2=30, minRadius=10, maxRadius=100)

        if blue_circles is not None:
            blue_circles = np.uint16(np.around(blue_circles))
            for circle in blue_circles[0, :]:
                x, y, radius = circle
                cv2.circle(frame, (x, y), radius, (255, 0, 0), 2)
                cv2.circle(frame, (x, y), 2, (255, 0, 0), 3)

        # Detect circles within the orange mask
        orange_circles = cv2.HoughCircles(orange_mask, cv2.HOUGH_GRADIENT, dp=1, minDist=30,
                                          param1=50, param2=30, minRadius=10, maxRadius=100)

        if orange_circles is not None:
            orange_circles = np.uint16(np.around(orange_circles))
            for circle in orange_circles[0, :]:
                x, y, radius = circle
                cv2.circle(frame, (x, y), radius, (0, 165, 255), 2)
                cv2.circle(frame, (x, y), 2, (0, 165, 255), 3)

        # Detect circles within the black mask
        black_circles = cv2.HoughCircles(black_mask, cv2.HOUGH_GRADIENT, dp=1, minDist=30,
                                         param1=50, param2=30, minRadius=10, maxRadius=100)

        if black_circles is not None:
            black_circles = np.uint16(np.around(black_circles))
            for circle in black_circles[0, :]:
                x, y, radius = circle
                cv2.circle(frame, (x, y), radius, (0, 0, 0), 2)
                cv2.circle(frame, (x, y), 2, (0, 0, 0), 3)

        # Write the frame to the video
        output_writer.write(frame)

        # Display the frame
        cv2.imshow("Circle Detection", frame)
        
        # Exit the loop if the 'q' key is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    else:
        print("Error capturing frame.")
        break

# Release the webcam, close OpenCV windows, and release the video writer
cap.release()
cv2.destroyAllWindows()
output_writer.release()
