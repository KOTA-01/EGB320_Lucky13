import cv2
import numpy as np

# Initialize the camera capture.
cap = cv2.VideoCapture(0)  # 0 represents the default camera, change it if you have multiple cameras.

# Define lower and upper bounds for Black color in HSV
lower_bound = np.array([0, 0, 0])
upper_bound = np.array([179, 255, 50])

while True:
    # Capture a frame from the camera.
    ret, frame = cap.read()

    if not ret:
        break
    # Convert the frame to HSV color space
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Create a mask for the black color
    black_mask = cv2.inRange(hsv_frame, lower_bound, upper_bound)

    # Create a masked view using bitwise operation
    masked_view = cv2.bitwise_and(frame, frame, mask=black_mask)

    # Convert the frame to grayscale.
    gray = cv2.cvtColor(masked_view, cv2.COLOR_BGR2GRAY)

    # Blur using a 3x3 kernel.
    gray_blurred = cv2.blur(gray, (3, 3))

    # Apply Hough transform on the blurred frame.
    detected_circles = cv2.HoughCircles(
        gray,
        cv2.HOUGH_GRADIENT,
        1,
        20,
        param1=50,
        param2=30,
        minRadius=1,
        maxRadius=40
    )

    # Draw circles that are detected.
    if detected_circles is not None:
        # Convert the circle parameters a, b, and r to integers.
        detected_circles = np.uint16(np.around(detected_circles))

        for pt in detected_circles[0, :]:
            a, b, r = pt[0], pt[1], pt[2]

            # Draw the circumference of the circle.
            cv2.circle(frame, (a, b), r, (0, 255, 0), 2)

            # Draw a small circle (of radius 1) to show the center.
            cv2.circle(frame, (a, b), 1, (0, 0, 255), 3)

    # Display the frame with detected circles.
    cv2.imshow("Detected Circles", frame)
    cv2.imshow("GrayScale", gray_blurred)

    # Exit the loop when the 'q' key is pressed.
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the camera and close the window.
cap.release()
cv2.destroyAllWindows()
