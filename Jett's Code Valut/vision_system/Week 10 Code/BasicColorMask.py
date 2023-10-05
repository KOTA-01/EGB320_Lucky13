import cv2
import numpy as np

# Define color ranges
color_ranges = {
    'yellow': (np.array([17, 3, 130]), np.array([40, 255, 255])),
    # 'black': (np.array([0, 0, 0]), np.array([179, 80, 55])),
    'blue': (np.array([90, 92, 0]), np.array([120, 255, 255])),
    'green': (np.array([33, 0, 102]), np.array([93, 123, 255])),
    'red': (np.array([0, 68, 94]), np.array([10, 255, 255]))
    # Add more color ranges here if needed
}

# Define colors for contour outlines corresponding to each color
contour_colors = {
    'yellow': (255, 255, 0),  # yellow
    'blue': (255, 0, 0),  # Blue
    'green': (0, 255, 0),  # Green
    'red': (0, 0, 255) # red
    # Add more colors here if needed
}


def detect_color_objects(frame, color_range, contour_color):
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv_frame, color_range[0], color_range[1])

    # Apply Gaussian blur to the mask
    mask = cv2.GaussianBlur(mask, (5, 5), 0)

    # # Apply morphological operations based on the color
    # if np.array_equal(color_range[0], np.array([0, 0, 0])):
    #     # For black color, use dilation to create a more circular look
    #     kernel = np.ones((10, 10), np.uint8)
    #     mask = cv2.dilate(mask, kernel, iterations=2)
    # else:
    #     # For blue and green, use erosion to emphasize straight lines
    # kernel = np.ones((5, 5), np.uint8)
    # mask = cv2.erode(mask, kernel, iterations=1)

    # Find contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Draw contour outlines with specified color
    for contour in contours:
        if cv2.contourArea(contour) > 10000:  # Adjust this value for minimum object size
            cv2.drawContours(frame, [contour], -1, contour_color, 2)

    return frame, mask 

# Open the camera
cap = cv2.VideoCapture(0)  # Use 0 for the default camera, you can change it if needed

# Create separate windows for each color mask
cv2.namedWindow('Yellow Mask')
cv2.namedWindow('Blue Mask')
cv2.namedWindow('Green Mask')
cv2.namedWindow('Red Mask')

while True:
    # Read a frame from the camera
    ret, frame = cap.read()
    og = frame
    if not ret:
        break

    # Detect objects of specific colors and get the masks
    masks = {}
    for color_name, color_range in color_ranges.items():
        contour_color = contour_colors[color_name]
        frame, mask = detect_color_objects(frame, color_range, contour_color)
        masks[color_name] = mask
   
    # Display the frame with contour outlines
    cv2.imshow('Object Detection', frame)
    cv2.imshow('source', og)
    # Display the masks in separate windows
    cv2.imshow('Yellow Mask', masks['yellow'])
    cv2.imshow('Blue Mask', masks['blue'])
    cv2.imshow('Green Mask', masks['green'])
    cv2.imshow('Red Mask', masks['red'])

    # Exit the loop if the 'q' key is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the camera and close all OpenCV windows
cap.release()
cv2.destroyAllWindows()
