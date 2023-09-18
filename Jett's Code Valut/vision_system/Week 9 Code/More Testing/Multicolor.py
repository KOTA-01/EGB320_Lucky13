import cv2
import numpy as np

# Define color ranges for object detection (in HSV format)
import cv2
import numpy as np

# Define color ranges for object detection (in HSV format)
color_ranges = {
        'black': (np.array([0, 0, 0]), np.array([179, 80, 55])),
        'blue':(np.array([0, 0, 20]), np.array([40, 255, 190])),
        'green': (np.array([20,0,38]), np.array([70,255,120]))
        # Add more color ranges here if needed
    }

# Define colors for contour outlines corresponding to each color
contour_colors = {
    'black': (0, 0, 255),  # Red for black
    'blue': (255, 0, 0),  # Blue
    'green': (0, 255, 0),  # Green
    # Add more colors here if needed
}

def detect_objects(frame):
    # Convert the frame to HSV color space
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    detected_objects = []

    for color, (lower, upper) in color_ranges.items():
        # Create a mask for the specific color
        mask = cv2.inRange(hsv_frame, lower, upper)

        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            # Calculate the area of the contour
            area = cv2.contourArea(contour)

            # Ignore small noise by setting a threshold for the area
            if area > 100:
                # Get the contour color
                contour_color = contour_colors[color]

                # Draw contours with the corresponding color
                cv2.drawContours(frame, [contour], -1, contour_color, 2)

    return frame, detected_objects

def get_object_type(color):
    # You can define a mapping from color to object type here
    object_types = {
        "red": "Apple",
        "green": "Leaf",
        "blue": "Blueberry",
        "yellow": "Banana",
    }
    return object_types.get(color, "Unknown")

def calculate_distance(object_width):
    # Implement your distance calculation logic based on object size here
    # This will depend on the camera's characteristics and calibration
    # For simplicity, this example assumes a linear relationship between size and distance
    return 100 / object_width  # Adjust the constant for your setup

# Open a video capture stream (you can also use a camera)
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    processed_frame, objects = detect_objects(frame)

    # Display the frame with bounding boxes
    cv2.imshow('Object Detection', processed_frame)

    # Press 'q' to exit the loop
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the video capture and close all OpenCV windows
cap.release()
cv2.destroyAllWindows()

def detect_objects(frame):
    # Convert the frame to HSV color space
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    detected_objects = []

    for color, (lower, upper) in color_ranges.items():
        # Create a mask for the specific color
        mask = cv2.inRange(hsv_frame, lower, upper)

        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            # Calculate the area of the contour
            area = cv2.contourArea(contour)

            # Ignore small noise by setting a threshold for the area
            if area > 100:
                # Get the bounding box coordinates
                x, y, w, h = cv2.boundingRect(contour)

                # Draw a bounding box around the object
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

                # Calculate the distance (you may need additional hardware for accurate distance measurement)
                distance = calculate_distance(w)  # Implement your distance calculation function

                detected_objects.append({
                    "color": color,
                    "type": get_object_type(color),
                    "distance": distance
                })

    return frame, detected_objects

def get_object_type(color):
    # You can define a mapping from color to object type here
    object_types = {
        "red": "Apple",
        "green": "Leaf",
        "blue": "Blueberry",
        "yellow": "Banana",
    }
    return object_types.get(color, "Unknown")

def calculate_distance(object_width):
    # Implement your distance calculation logic based on object size here
    # This will depend on the camera's characteristics and calibration
    # For simplicity, this example assumes a linear relationship between size and distance
    return 100 / object_width  # Adjust the constant for your setup

# Open a video capture stream (you can also use a camera)
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    processed_frame, objects = detect_objects(frame)

    # Display the frame with bounding boxes
    cv2.imshow('Object Detection', processed_frame)

    # Press 'q' to exit the loop
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the video capture and close all OpenCV windows
cap.release()
cv2.destroyAllWindows()
