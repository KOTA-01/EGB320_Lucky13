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
output_writer = cv2.VideoWriter('circle_and_shape_detection_output.avi', cv2.VideoWriter_fourcc(*'XVID'), 20.0, (640, 480))

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

        # Process each color separately
        for color_contours, color in zip([blue_contours, orange_contours, black_contours], [(255, 0, 0), (0, 165, 255), (0, 0, 0)]):
            shape_count = {}  # Dictionary to store shape counts
            for contour in color_contours:
                # Approximate the shape of the contour
                epsilon = 0.04 * cv2.arcLength(contour, True)
                approx = cv2.approxPolyDP(contour, epsilon, True)
                
                # Identify shapes based on the number of vertices
                num_vertices = len(approx)
                shape = "Unidentified"
                if num_vertices == 3:
                    shape = "Triangle"
                elif num_vertices == 4:
                    shape = "Rectangle"
                elif num_vertices == 5:
                    shape = "Pentagon"
                elif num_vertices == 6:
                    shape = "Hexagon"
                elif num_vertices < 6:
                    shape = "circle"
                
                # Calculate the contour area
                contour_area = cv2.contourArea(contour)
                
                # Filter out small contours based on area
                if 1000 < contour_area < 20000:
                    # Increment shape count
                    shape_count[shape] = shape_count.get(shape, 0) + 1
                    
                    # Draw contours and add text label
                    cv2.drawContours(frame, [approx], -1, color, 2)
                    x, y = approx[0][0]
                    cv2.putText(frame, shape, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

           


            # Display shape counts above the mask
            y_offset = 30
            for shape, count in shape_count.items():
                cv2.putText(frame, f"{shape}: {count}", (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                y_offset += 30

        # Write the frame to the video
        output_writer.write(frame)

        # Display the frame
        cv2.imshow("Circle and Shape Detection", frame)
        
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
