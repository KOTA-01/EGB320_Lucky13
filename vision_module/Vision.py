import cv2
import numpy as np
# from picamera2 import Picamera2
# from libcamera import controls
import time

class Vision:
    def __init__(self) :
        # #Set camera Properties
        # self.cap = Picamera2()  
        # self.config = self.cap.create_video_configuration(main={"format":'XRGB8888',"size":(1640,1232)})
        # self.cap.configure(self.config)
        # self.cap.align_configuration(self.config)
        # self.cap.configure(self.config)
        # self.cap.set_controls({"AwbEnable": False, "ExposureTime": 1000, "AnalogueGain": 2.0})
        # self.cap.start()

        self.cap = cv2.VideoCapture(0)
        # self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 308)
        # self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 410)
        # Constants
        self.MIN_CONTOUR_AREA_THRESHOLD = 500
        # Libs
        self.detected_objects_count = {}
        self.masks = {}
        self.center_points_dict = {}


    #Note need to be chnaged
    color_ranges = {
        'wall': (np.array([39, 0, 0]), np.array([162, 255, 255])),
        'yellow': (np.array([9, 85, 0]), np.array([19  , 255, 255])),
        #'blue': (np.array([39, 0, 0]), np.array([162, 255, 255])),
        'green': (np.array([33, 0, 0]), np.array([94, 255, 255])),
        'orange': (np.array([0, 0, 157]), np.array([11, 255, 255])),
        'black': (np.array([0, 0, 43]), np.array([179, 55, 109]))
        
    }

    contour_colors = {
        'wall': (128, 0, 128),
        'yellow': (255, 255, 0),
        #'blue': (255, 0, 0),
        'green': (0, 128, 0),
        'orange': (255, 200, 0),
        'black': (0, 0, 0)
        
    }




    

    def detect_color_objects(self, frame, color_range, contour_color, color_name):
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv_frame, color_range[0], color_range[1])

        # Create a mask to exclude the top 100 pixels
        mask[:100, :] = 0

        # Preprocess the mask
        mask = cv2.GaussianBlur(mask, (5, 5), 0)
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=2)
        mask = cv2.dilate(mask, kernel, iterations=2)

        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Draw a line to indicate the border
        cv2.line(frame, (0, 100), (frame.shape[1], 100), (0, 0, 255), 2)

        # Draw contour outlines and calculate center points
        detected_objects = 0
        center_points = []
        estimated_distances = []
        # Initialize variables for average top position calculation
        top_positions = []

        for contour in contours:
            if cv2.contourArea(contour) > self.MIN_CONTOUR_AREA_THRESHOLD:
                cv2.drawContours(frame, [contour], -1, contour_color, 2)
                detected_objects += 1

                if color_name == 'wall':
                    # Get the topmost point of the contour
                    top_point = tuple(contour[contour[:, :, 1].argmin()][0])
                    # Check if the top point falls within the specified x-range
                    if (frame.shape[1] // 3) <= top_point[0] <= ((frame.shape[1] // 3) * 2):
                        if top_point[1] < (frame.shape[0] // 2):
                            top_positions.append(top_point[1])  # Y-coordinate

                            # Draw a circle at the top point
                            cv2.circle(frame, top_point, 5, contour_color, -1)

                            # Draw a line across the top point, always across the x-axis
                            cv2.line(frame, (0, top_point[1]), (frame.shape[1], top_point[1]), contour_color, 1)

                            # Show Y-coordinate numbers on the Y-axis
                            cv2.putText(frame, str(top_point[1]), (10, top_point[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, contour_color, 2)


                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                    center_points.append((cX, cY))

                    if color_name == 'black':
                        rect = cv2.minAreaRect(contour)
                        width = max(rect[1]) if rect[1][0] > rect[1][1] else rect[1][1]
                        estimated_distance = self.black_pixel_to_distance(width)
                        estimated_distances.append(estimated_distance)
                        estimated_distance_text = f'Distance: {estimated_distance:.2f}'
                        cv2.putText(frame, estimated_distance_text, (cX - 50, cY + 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, contour_color, 2)

                    if color_name == 'green':
                        rect = cv2.minAreaRect(contour)
                        width = max(rect[1]) if rect[1][0] > rect[1][1] else rect[1][1]
                        estimated_distance = self.green_pixel_to_distance(width)
                        estimated_distances.append(estimated_distance)
                        estimated_distance_text = f'Distance: {estimated_distance:.2f}'
                        cv2.putText(frame, estimated_distance_text, (cX - 50, cY + 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, contour_color, 2)

        return frame, mask, detected_objects, center_points

    def visualize(self, frame, masks):
        cv2.imshow('Object Detection', frame)
        cv2.imshow('Yellow Mask', masks['yellow'])
        #cv2.imshow('Blue Mask', masks['blue'])
        cv2.imshow('Green Mask', masks['green'])
        cv2.imshow('Orange Mask', masks['orange'])
        cv2.imshow('Black Mask', masks['black'])
        cv2.imshow('Wall Mask', masks['wall'])

    def distance(self):    
        # Simulated data: Distance (in cm) and corresponding pixel counts (For Black)
        black_distances = [15, 20, 25, 30]
        black_pixel_width = [441, 335, 271, 232]
        black_coefficients = np.polyfit(black_pixel_width, black_distances, 2)
        self.black_pixel_to_distance = np.poly1d(black_coefficients)

        # Simulated data: Distance (in cm) and corresponding pixel counts (For Green)
        green_distances = [15, 20, 25, 30]
        green_pixel_width = [441, 335, 271, 232]
        green_coefficients = np.polyfit(green_pixel_width, green_distances, 2)
        self.green_pixel_to_distance = np.poly1d(green_coefficients)

    def find_infomation(self):
        self.distance()
        while True:
            ret, frame = self.cap.read()
            if not ret:
                break

            for color_name, color_range in self.color_ranges.items():
                contour_color = self.contour_colors[color_name]
                frame, mask, num_objects, center_points = self.detect_color_objects(frame, color_range, contour_color, color_name)
                self.masks[color_name] = mask
                self.detected_objects_count[color_name] = num_objects
                self.center_points_dict[color_name] = center_points

            for idx, (color_name, count) in enumerate(self.detected_objects_count.items()):
                text = f'{color_name.capitalize()}: {count}'
                cv2.putText(frame, text, (20, 30 + idx * 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

            for color_name, center_points in self.center_points_dict.items():
                for cX, cY in center_points:
                    text = f'({frame.shape[1] // 2 - cX})'
                    cv2.circle(frame, (cX, cY), 5, self.contour_colors[color_name], -1)
                    cv2.putText(frame, text, (cX + 10, cY), cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.contour_colors[color_name], 2)

        
            self.visualize(frame, self.masks)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        self.cap.release()
        cv2.destroyAllWindows()

    def release_camera(self):
        self.cap.release()

if __name__ == "__main__":
    vision = Vision()
    vision.find_infomation()
    vision.release_camera()

