import cv2
import numpy as np

# Function to create a mask for the dominant color range
def create_dominant_color_mask(frame, color_ranges, dominant_color):
    # Convert the frame to the HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Get the dominant color range based on the provided color name
    lower_color, upper_color = color_ranges[dominant_color]

    # Create a mask for the dominant color range
    mask = cv2.inRange(hsv, np.array(lower_color), np.array(upper_color))

    return mask

# Function to identify color borders in the frame and count pixels in each color range
def identify_color_borders(frame, color_ranges):
    # Convert the frame to the HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Initialize a dictionary to store pixel counts for each color range
    color_counts = {color_name: 0 for color_name in color_ranges}

    for color_name, (lower_color, upper_color) in color_ranges.items():
        # Define a mask to isolate the specified color range
        mask = cv2.inRange(hsv, np.array(lower_color), np.array(upper_color))

        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Count the number of pixels in the color range
        pixel_count = np.sum(mask > 0)
        
        # Update the pixel count for the current color range
        color_counts[color_name] = pixel_count

        # Draw the color borders on the original frame
        cv2.drawContours(frame, contours, -1, (0, 255, 0), 2)

        # Add text to label the color and pixel count
        cv2.putText(frame, f'{color_name}: {pixel_count}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

    # Find the most dominant color range based on pixel count
    dominant_color = max(color_counts, key=color_counts.get)

    return frame, dominant_color

# Define the color ranges for major colors in HSV format
color_ranges = {
    'red': ([0, 100, 100], [10, 255, 255]),
    'green': ([35, 100, 100], [85, 255, 255]),
    'blue': ([100, 100, 100], [130, 255, 255]),
    'yellow': ([20, 100, 100], [35, 255, 255]),
    'black': ([0, 0, 0], [180, 255, 30]),
    #'gray': ([0, 0, 80], [180, 40, 255]),
}

# Initialize the camera capture
cap = cv2.VideoCapture(0)  # 0 represents the default camera (you can change this to the camera you want to use)

while True:
    # Read a frame from the camera
    ret, frame = cap.read()

    if not ret:
        break

    # Identify color borders and count pixels in each color range
    frame, dominant_color = identify_color_borders(frame, color_ranges)

    # Create a mask for the dominant color range
    dominant_color_mask = create_dominant_color_mask(frame, color_ranges, dominant_color)

    # Display the resulting frame with color borders
    cv2.imshow('Color Borders Detection', frame)

    # Display the dominant color mask in a separate frame
    cv2.imshow('Dominant Color Mask', dominant_color_mask)

    # Exit the loop when the 'q' key is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the camera and close all OpenCV windows
cap.release()
cv2.destroyAllWindows()
