import cv2
import numpy as np

# Define color ranges
color_ranges = {
    'black': (np.array([0, 0, 0]), np.array([179, 80, 55])),
    'blue': (np.array([0, 0, 20]), np.array([40, 255, 190])),
    'green': (np.array([20, 0, 38]), np.array([70, 255, 120]))
    # Add more color ranges here if needed
}

# Define colors for contour outlines corresponding to each color
contour_colors = {
    'black': (0, 0, 255),  # Red for black
    'blue': (255, 0, 0),  # Blue
    'green': (0, 255, 0),  # Green
    # Add more colors here if needed
}

def detect_color_objects(frame, color_range, contour_color):
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv_frame, color_range[0], color_range[1])

    # Apply Gaussian blur to the mask
    mask = cv2.GaussianBlur(mask, (5, 5), 0)

    # Apply morphological operations based on the color
    if np.array_equal(color_range[0], np.array([0, 0, 0])):
        # For black color, use dilation to create a more circular look
        kernel = np.ones((10, 10), np.uint8)
        mask = cv2.dilate(mask, kernel, iterations=2)
    else:
        # For blue and green, use erosion to emphasize straight lines
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=1)

    # Find contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Draw contour outlines with specified color
    for contour in contours:
        if cv2.contourArea(contour) > 100:  # Adjust this value for minimum object size
            cv2.drawContours(frame, [contour], -1, contour_color, 2)

    return frame


# Open the camera
cap = cv2.VideoCapture(0)  # Use 0 for the default camera, you can change it if needed

while True:
    # Read a frame from the camera
    ret, frame = cap.read()
    if not ret:
        break

    # Detect objects of specific colors and draw contour outlines
    for color_name, color_range in color_ranges.items():
        contour_color = contour_colors[color_name]
        frame = detect_color_objects(frame, color_range, contour_color)

    # Display the frame
    cv2.imshow('Object Detection', frame)

    # Exit the loop if the 'q' key is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the camera and close all OpenCV windows
cap.release()
cv2.destroyAllWindows()

