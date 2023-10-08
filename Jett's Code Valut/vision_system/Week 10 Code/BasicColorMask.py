import cv2
import numpy as np

# Define color ranges
color_ranges = {
    'yellow': (np.array([17, 3, 110]), np.array([40, 255, 255])),
    # 'black': (np.array([0, 0, 0]), np.array([179, 80, 55])),
    'blue': (np.array([100, 92, 0]), np.array([120, 255, 255])),
    'green': (np.array([30, 0, 55]), np.array([93, 123, 255])),
    'red': (np.array([0, 68, 80]), np.array([10, 255, 255]))
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
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.erode(mask, kernel, iterations=2)
    mask = cv2.dilate(mask, kernel, iterations=2)

    # Find contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Draw contour outlines with specified color
    detected_objects = 0  # Count of detected objects
    for contour in contours:
        if cv2.contourArea(contour) > 5000:  # Adjust this value for minimum object size
            cv2.drawContours(frame, [contour], -1, contour_color, 2)
            detected_objects += 1

    return frame, mask, detected_objects


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
    if not ret:
        break

    detected_objects_count = {}  # Dictionary to store the count of detected objects for each color
    # Detect objects of specific colors and get the masks
    masks = {}
    for color_name, color_range in color_ranges.items():
        contour_color = contour_colors[color_name]
        frame, mask, num_objects= detect_color_objects(frame, color_range, contour_color)
        masks[color_name] = mask
        detected_objects_count[color_name] = num_objects 
    # Display the count of detected objects in a text box
    for idx, (color_name, count) in enumerate(detected_objects_count.items()):
        text = f'{color_name.capitalize()}: {count}'
        cv2.putText(frame, text, (20, 30 + idx * 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

   
    # Display the frame with contour outlines
    cv2.imshow('Object Detection', frame)
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
