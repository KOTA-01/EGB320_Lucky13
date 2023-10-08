import cv2
import numpy as np

# Initialize the webcam or video capture device

cap = cv2.VideoCapture(0)  # 0 for the default camera
# # Set camera properties
# cap.set(cv2.CAP_PROP_FRAME_WIDTH, 308)
# cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 410)


while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Convert the frame to the HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Define a lower and upper threshold for detecting red color
    lower_red = np.array([0, 100, 100])
    upper_red = np.array([10, 255, 255])

    # Create a mask to isolate red pixels
    mask = cv2.inRange(hsv, lower_red, upper_red)

    # Find contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Initialize variables for the largest width and largest contour
    largest_width = 0
    largest_contour = None

    if len(contours) > 0:
        # Iterate through all detected contours
        for contour in contours:
            # Calculate the bounding rectangle for the contour
            x, y, w, h = cv2.boundingRect(contour)

            # Check if the contour is large enough (you can set a threshold)
            if w > 10 and h > 10:
                # Calculate the width of the contour
                width = w

                # If this width is larger than the largest width found so far, update it
                if width > largest_width:
                    largest_width = width
                    largest_contour = contour

        # Draw the largest contour on the frame
        if largest_contour is not None:
            cv2.drawContours(frame, [largest_contour], -1, (0, 255, 0), 2)

    # Display the largest width on the frame
    cv2.putText(frame, f"Largest Width: {largest_width}px", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    # Display the frame
    cv2.imshow('Largest Width Detection', frame)

    # Exit the loop when 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the video capture and close all OpenCV windows
cap.release()
cv2.destroyAllWindows()
