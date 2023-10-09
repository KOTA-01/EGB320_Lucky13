import cv2
import numpy as np
#import picamera2
import time

class Vision:
    def __init__(self):
        # Set camera Properties
        # self.cap = picamera2.Picamera2()  
        # self.config = self.cap.create_video_configuration(main={"format":'XRGB8888',"size":(1640,1232)})
        # self.cap.configure(self.config)
        # self.cap.start()

        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 308)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 410)

        # Define lower and upper bounds for Black color in HSV
        self.lower_bound = np.array([0, 0, 0])
        self.upper_bound = np.array([179, 80, 55])

        # Simulated data: Distance (in cm) and corresponding pixel counts
        distances = [20, 25, 30, 35, 40, 45, 65]
        pixel_width = [178, 146, 119, 102, 86, 78, 54]

        # Fit a polynomial regression model for pixel-to-distance mapping
        coefficients = np.polyfit(pixel_width, distances, 2)
        self.pixel_to_distance = np.poly1d(coefficients)

        # Initialize variables for calculating the center point of blobs
        self.all_blob_centroids = []

    def display(self):
        # Initialize windows
        cv2.namedWindow('Camera Feed')
        cv2.namedWindow('Masked View')

    def color_range(self):
        self.color_ranges = {
            # 'black': (np.array([0, 0, 0]), np.array([179, 80, 55])),
            # 'blue':(np.array([0, 0, 20]), np.array([40, 255, 190])),
            # 'green': (np.array([20,0,38]), np.array([70,255,120]))
            # # Add more color ranges here if needed
        }

        self.contour_colors = {
            'black': (0, 0, 255),  # Red for black
            'blue': (255, 0, 0),  # Blue
            'green': (0, 255, 0),  # Green
            # Add more colors here if needed
        }

    def find_information(self):
        # Main loop for processing video frames
        while True:
            # Capture a frame from the camera
            # frame = self.cap.capture_array()
            # frame = cv2.resize(frame, (640,480))

            ret, frame = self.cap.read()
            if not ret:
                return None

            # Convert the frame to HSV color space
            hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            # Create a mask for the black color
            black_mask = cv2.inRange(hsv_frame, self.lower_bound, self.upper_bound)

            # Find contours in the mask
            contours, _ = cv2.findContours(black_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # Create a masked view using bitwise operation
            masked_view = cv2.bitwise_and(frame, frame, mask=black_mask)

            # Initialize a count for circular black blobs
            circular_blob_count = 0

            # Initialize a list to store the centroids of circular blobs in the current frame
            frame_blob_centroids = []

            # Initialize an empty list to store estimated distances for the current frame
            estimated_distances = []
            pixel_widths = [] 

            # Iterate through the detected contours
            for contour in contours:
                # Calculate the area of the contour
                area = cv2.contourArea(contour)
                # Define a threshold for considering a contour as a circular blob
                min_blob_area = 1000  # Adjust this threshold as needed

                if area >= min_blob_area:
                    # Find the centroid (center) of the current contour
                    M = cv2.moments(contour)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        centroid = (cx, cy)
                        
                        # Find the bounding rectangle of the largest contour
                        x, y, w, h = cv2.boundingRect(contour)

                        # Check if the bounding rectangle touches the edge of the frame
                        edge_touching = x == 0 or y == 0 or x + w == frame.shape[1] or y + h == frame.shape[0]

                        if edge_touching:
                            # Draw a red rectangle around the largest contour
                            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
                            cv2.putText(frame, "Edge Touching", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                        else: 
                            # Append the centroid to the list of centroids for this frame
                            frame_blob_centroids.append(centroid)

                            # Increment the circular blob count
                            circular_blob_count += 1

            # Calculate the centroid of all circular blobs in the current frame
            if frame_blob_centroids:
                frame_centroid_x = int(np.mean([cx for (cx, cy) in frame_blob_centroids]) - (frame.shape[1] / 2))
                frame_centroid_y = int(np.mean([cy for (cx, cy) in frame_blob_centroids]))

                # Calculate and display the estimated distance for each circular blob
                for contour in contours:
                    area = cv2.contourArea(contour)
                    if area >= min_blob_area:
                        # Calculate blob width
                        x, y, w, h = cv2.boundingRect(contour)
                        blob_width = w
                        pixel_widths.append(blob_width)

                        # Estimate the distance using pixel-to-distance mapping
                        estimated_distance = self.pixel_to_distance(blob_width)
                        estimated_distances.append(estimated_distance)
            else:
                frame_centroid_x = 0
                frame_centroid_y = 0
                
            # Calculate the average estimated distance for the current frame
            if estimated_distances: 
                average_distance = sum(estimated_distances) / len(estimated_distances)
            else:
                average_distance = 0
            # Return the relevant information as a dictionary
            info = {
                "blob_count": circular_blob_count,
                "average_distance": average_distance,
                "frame_centroid_x": frame_centroid_x,
                "frame_centroid_y": frame_centroid_y,
            }
            return info
            

    def realse_camera(self):
        # Release the camera and close all windows
        self.cap.release()
