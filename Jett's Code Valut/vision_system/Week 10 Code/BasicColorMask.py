import cv2
import numpy as np

# Constants
MIN_CONTOUR_AREA_THRESHOLD = 5000

# Color Ranges and Contour Colors (As in the original code)
color_ranges = {
    'yellow': (np.array([13, 0, 60]), np.array([40, 255, 255])),
    'blue': (np.array([100, 92, 0]), np.array([120, 255, 255])),
    'green': (np.array([30, 0, 15]), np.array([74, 123, 255])),
    'orange': (np.array([0, 0, 44]), np.array([10, 255, 255])),
    'black': (np.array([0, 0, 44]), np.array([10, 255, 255])),
    'wall': (np.array([0, 0, 44]), np.array([10, 255, 255]))
}

contour_colors = {
    'yellow': (255, 255, 0),
    'blue': (255, 0, 0),
    'green': (0, 128, 0),
    'orange': (255, 200, 0),
    'black': (0, 0, 0),
    'wall': (128, 0, 128)
}

def detect_color_objects(frame, color_range, contour_color, color_name):
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv_frame, color_range[0], color_range[1])

    # Preprocess the mask
    mask = cv2.GaussianBlur(mask, (5, 5), 0)
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.erode(mask, kernel, iterations=2)
    mask = cv2.dilate(mask, kernel, iterations=2)

    # Find contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Draw contour outlines and calculate center points
    detected_objects = 0
    center_points = []
    estimated_distances = []

    for contour in contours:
        if cv2.contourArea(contour) > MIN_CONTOUR_AREA_THRESHOLD:
            cv2.drawContours(frame, [contour], -1, contour_color, 2)
            detected_objects += 1

            M = cv2.moments(contour)
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                center_points.append((cX, cY))

                if color_name == 'black':
                    rect = cv2.minAreaRect(contour)
                    width = max(rect[1]) if rect[1][0] > rect[1][1] else rect[1][1]
                    estimated_distance = black_pixel_to_distance(width)
                    estimated_distances.append(estimated_distance)
                    estimated_distance_text = f'Distance: {estimated_distance:.2f}'
                    cv2.putText(frame, estimated_distance_text, (cX - 50, cY + 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, contour_color, 2)

                # if color_name == 'orange':
                #     rect = cv2.minAreaRect(contour)
                #     width = max(rect[1]) if rect[1][0] > rect[1][1] else rect[1][1]
                #     estimated_distance = orange_pixel_to_distance(width)
                #     estimated_distances.append(estimated_distance)
                #     estimated_distance_text = f'Distance: {estimated_distance:.2f}'
                #     cv2.putText(frame, estimated_distance_text, (cX - 50, cY + 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, contour_color, 2)

                if color_name == 'green':
                    rect = cv2.minAreaRect(contour)
                    width = max(rect[1]) if rect[1][0] > rect[1][1] else rect[1][1]
                    estimated_distance = green_pixel_to_distance(width)
                    estimated_distances.append(estimated_distance)
                    estimated_distance_text = f'Distance: {estimated_distance:.2f}'
                    cv2.putText(frame, estimated_distance_text, (cX - 50, cY + 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, contour_color, 2)

    return frame, mask, detected_objects, center_points

def main():
    # Initialize the camera and other variables
    cap = cv2.VideoCapture(0)

    # Simulated data: Distance (in cm) and corresponding pixel counts (For Black)
    black_distances = [15, 20, 25, 30]
    black_pixel_width = [441, 335, 271, 232]
    black_coefficients = np.polyfit(black_pixel_width, black_distances, 2)
    global black_pixel_to_distance
    black_pixel_to_distance = np.poly1d(black_coefficients)

    # # Simulated data: Distance (in cm) and corresponding pixel counts (For Orange)
    # orange_distances = [15, 20, 25, 30]
    # orange_pixel_width = [441, 335, 271, 232]
    # orange_coefficients = np.polyfit(orange_pixel_width, orange_distances, 2)
    # global orange_pixel_to_distance
    # orange_pixel_to_distance = np.poly1d(orange_coefficients)

    # Simulated data: Distance (in cm) and corresponding pixel counts (For Green)
    green_distances = [15, 20, 25, 30]
    green_pixel_width = [441, 335, 271, 232]
    green_coefficients = np.polyfit(green_pixel_width, green_distances, 2)
    global green_pixel_to_distance
    green_pixel_to_distance = np.poly1d(green_coefficients)

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        detected_objects_count = {}
        masks = {}
        center_points_dict = {}

        for color_name, color_range in color_ranges.items():
            contour_color = contour_colors[color_name]
            frame, mask, num_objects, center_points = detect_color_objects(frame, color_range, contour_color, color_name)
            masks[color_name] = mask
            detected_objects_count[color_name] = num_objects
            center_points_dict[color_name] = center_points

        for idx, (color_name, count) in enumerate(detected_objects_count.items()):
            text = f'{color_name.capitalize()}: {count}'
            cv2.putText(frame, text, (20, 30 + idx * 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

        for color_name, center_points in center_points_dict.items():
            for cX, cY in center_points:
                text = f'({frame.shape[1] // 2 - cX})'
                cv2.circle(frame, (cX, cY), 5, contour_colors[color_name], -1)
                cv2.putText(frame, text, (cX + 10, cY), cv2.FONT_HERSHEY_SIMPLEX, 0.5, contour_colors[color_name], 2)

        cv2.imshow('Object Detection', frame)
        cv2.imshow('Yellow Mask', masks['yellow'])
        cv2.imshow('Blue Mask', masks['blue'])
        cv2.imshow('Green Mask', masks['green'])
        cv2.imshow('Orange Mask', masks['orange'])
        cv2.imshow('Black Mask', masks['black'])
        cv2.imshow('Wall Mask', masks['wall'])

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
