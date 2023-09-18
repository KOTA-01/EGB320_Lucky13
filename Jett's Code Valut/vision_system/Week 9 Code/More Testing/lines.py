import cv2
import numpy as np

# Define color ranges for object detection (in HSV format)
color_ranges = {
    "blue":(np.array([0, 0, 20]), np.array([40, 255, 190])),
    "green":(np.array([20,0,38]), np.array([70,255,120])),
}

def detect_straight_edges(frame):
    # Convert the frame to HSV color space
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    detected_edges = []
    masked_views = {}

    for color, (lower, upper) in color_ranges.items():
        # Create a mask for the specific color
        mask = cv2.inRange(hsv_frame, lower, upper)

        # Find edges using Canny edge detection
        edges = cv2.Canny(mask, 50, 150)

        # Find straight lines in the edges using Hough Line Transform
        lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=50, minLineLength=50, maxLineGap=5)

        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                detected_edges.append({
                    "color": color,
                    "start_point": (x1, y1),
                    "end_point": (x2, y2)
                })

        # Store the masked view for this color
        masked_views[color] = cv2.bitwise_and(frame, frame, mask=mask)

    return frame, detected_edges, masked_views

# Open a video capture stream (you can also use a camera)
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    processed_frame, edges, masked_views = detect_straight_edges(frame)

    # Draw the detected edges on the frame
    for edge in edges:
        color = edge["color"]
        start_point = edge["start_point"]
        end_point = edge["end_point"]
        line_color = (0, 0, 255) if color == "blue" else (0, 255, 0)  # Blue for blue edges, Green for green edges
        cv2.line(processed_frame, start_point, end_point, line_color, 2)

    # Display the frame with detected edges
    cv2.imshow('Edge Detection', processed_frame)

    # Display masked views for blue and green
    cv2.imshow('Masked View Blue', masked_views["blue"])
    cv2.imshow('Masked View Green', masked_views["green"])

    # Press 'q' to exit the loop
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the video capture and close all OpenCV windows
cap.release()
cv2.destroyAllWindows()
