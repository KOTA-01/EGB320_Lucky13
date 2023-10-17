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
        self.MIN_CONTOUR_AREA_THRESHOLD = 10000

    color_ranges = {
        #'wall': (np.array([39, 0, 0]), np.array([162, 255, 255])),
        'yellow': (np.array([9, 85, 0]), np.array([19  , 255, 255])),
        'blue': (np.array([39, 0, 0]), np.array([162, 255, 255])),
        'green': (np.array([33, 0, 0]), np.array([94, 255, 255])),
        'orange': (np.array([0, 0, 157]), np.array([11, 255, 255])),
        'black': (np.array([0, 0, 43]), np.array([179, 55, 109]))
        
    }

    contour_colors = {
        #'wall': (128, 0, 128),
        'yellow': (255, 255, 0),
        'blue': (255, 0, 0),
        'green': (0, 128, 0),
        'orange': (255, 200, 0),
        'black': (0, 0, 0)
        
    }

    def detect_color_objects(self, frame, color_range):
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv_frame, color_range[0], color_range[1])

        # Create a mask to exclude the top 100 pixels
        mask[:100, 100:] = 0

        # Preprocess the mask
        mask = cv2.GaussianBlur(mask, (5, 5), 0)
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=2)
        mask = cv2.dilate(mask, kernel, iterations=2)

        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Draw a line to indicate the border
        cv2.line(frame, (0, 100), (frame.shape[1], 100), (0, 0, 255), 2)

        return contours, mask, frame

        # Draw contour outlines and calculate center points
        detected_objects = 0
        center_points = []
        estimated_distances = []
        top_positions = []
        estimated_distance = 0  # Default value

        for contour in contours:
            if cv2.contourArea(contour) > self.MIN_CONTOUR_AREA_THRESHOLD:
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
                        estimated_distance = self.black_pixel_to_distance(width)
                        estimated_distances.append(estimated_distance)
                        estimated_distance_text = f'Distance: {estimated_distance:.2f}'
                        cv2.putText(frame, estimated_distance_text, (cX - 50, cY + 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, contour_color, 2)

                    elif color_name == 'green':
                        rect = cv2.minAreaRect(contour)
                        width = max(rect[1]) if rect[1][0] > rect[1][1] else rect[1][1]
                        estimated_distance = self.green_pixel_to_distance(width)
                        estimated_distances.append(estimated_distance)
                        estimated_distance_text = f'Distance: {estimated_distance:.2f}'
                        cv2.putText(frame, estimated_distance_text, (cX - 50, cY + 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, contour_color, 2)
                    else: 
                        estimated_distance = 0

        return frame, mask, detected_objects, center_points, estimated_distance

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

    # def find_infomation(self):
    #     self.distance()
    #     while True:
    #         ret, frame = self.cap.read()
    #         if not ret:
    #             break
    #         formatted_info = []
    #         for color_name, color_range in self.color_ranges.items():
    #             contour_color = self.contour_colors[color_name]
    #             frame, mask, num_objects, center_points, estimated_distance = self.detect_color_objects(frame, color_range, contour_color, color_name)
    #             self.masks[color_name] = mask
    #             self.detected_objects_count[color_name] = num_objects
    #             self.center_points_dict[color_name] = center_points
    #             for cX, cY in center_points:
    #                     text = f'({frame.shape[1] // 2 - cX})'
    #                     cv2.circle(frame, (cX, cY), 5, self.contour_colors[color_name], -1)
    #                     cv2.putText(frame, text, (cX + 10, cY), cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.contour_colors[color_name], 2)
    #             if color_name == 'black':
    #                 formatted_info.append(f"({color_name}, {text}_degrees, {estimated_distance:.2f}cm), {self.detected_objects_count[color_name]}_Asile")
    #             else:
    #                 formatted_info.append(f"({color_name}, {text}_degrees, {estimated_distance:.2f}cm), {0}_Asile")      

    #         self.visualize(frame, self.masks)
    #         return formatted_info

    def release_camera(self):
        self.cap.release()
        cv2.destroyAllWindows()    

    def display(self):
        # Initialize windows
        cv2.namedWindow('Camera Feed')
        cv2.namedWindow('Masked View')

    def Test():

        return True
    
    def CheckPeople(self):
        while True:
            ret, frame = self.cap.read()
            if not ret:
                break
            Green_Contours, Green_Mask, Green_frame = self.detect_color_objects(frame, self.color_ranges.get('green'))
            People_detected = 0
            center_points = []
            bearing = []
            for contour in Green_Contours:
                if cv2.contourArea(contour) > self.MIN_CONTOUR_AREA_THRESHOLD:
                    cv2.drawContours(frame, [contour], -1, self.contour_colors.get('green'), 2)
                    People_detected += 1
                    # X,Y Coords
                    M = cv2.moments(contour)
                    if M["m00"] != 0:
                        cX = int(M["m10"] / M["m00"])
                        cY = int(M["m01"] / M["m00"])
                        cX = frame.shape[1] // 2 - cX
                        center_points.append((cX, cY))
                        bearing.append(cX)
                    # Bearing
            #View
            cv2.imshow('Camera Feed', Green_frame)
            cv2.imshow('Masked View', Green_Mask)

             # Check for the 'q' key to exit the loop
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            if (People_detected == 1):
                print("1 person detected [", cX, "]")
                return True, bearing
            elif (People_detected > 1):
                print("Too many People")
                return False
            else:    
                print("clear")
                return False            
        print("error")
        return False
        
    def Asile(self):
        while True:
            ret, frame = self.cap.read()
            if not ret:
                break
            Black_Contours, Black_Mask, Black_frame = self.detect_color_objects(frame, self.color_ranges.get('black'))
            Dots_detected = 0
            center_points = []
            bearing = []
            SumBearing = 0
            Black_distances = []
            for contour in Black_Contours:
                if cv2.contourArea(contour) > self.MIN_CONTOUR_AREA_THRESHOLD:
                    cv2.drawContours(frame, [contour], -1, self.contour_colors.get('black'), 2)
                    Dots_detected += 1
            # Distance
                    rect = cv2.minAreaRect(contour)
                    width = max(rect[1]) if rect[1][0] > rect[1][1] else rect[1][1]
                    estimated_distance = self.black_pixel_to_distance(width)
                    Black_distances.append(estimated_distance)
            # X,Y Coords
                    M = cv2.moments(contour)
                    if M["m00"] != 0:
                        cX = int(M["m10"] / M["m00"])
                        cY = int(M["m01"] / M["m00"])
                        cX = frame.shape[1] // 2 - cX
                        center_points.append((cX, cY))
            # Bearing
                        bearing.append(cX)
            for cX in bearing:
                    SumBearing += cX
            if SumBearing != 0:
                AvgBearing = round(SumBearing/len(bearing)) 
            else:
                AvgBearing = 0          
            
            SumDistance = sum(Black_distances)
            if SumDistance != 0:
                AvgDistance = round(SumDistance/len(Black_distances))
            else:
                AvgDistance = 0
            #View
            cv2.imshow('Camera Feed', Black_frame)
            cv2.imshow('Masked View', Black_Mask)

             # Check for the 'q' key to exit the loop
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            if (Dots_detected == 1):
                print("Asile1 [", AvgBearing, AvgDistance, "]")
                return True, 1, AvgBearing, AvgDistance
            elif (Dots_detected == 2):
                print("Asile2 [", AvgBearing, AvgDistance, "]")
                return True, 2, AvgBearing, AvgDistance
            elif (Dots_detected == 3):
                print("Asile3 [", AvgBearing, AvgDistance, "]")
                return True, 3, AvgBearing, AvgDistance
            else:    
                print("No Asile Found")
                return False
            
        print("error")
        return False
    
    def Shelves(self):
        while True:
                ret, frame = self.cap.read()
                if not ret:
                    break
                Blue_Contours, Blue_Mask, Blue_frame = self.detect_color_objects(frame, self.color_ranges.get('blue'))
                Shelves = 0
                center_points = []
                bearing = []
                for contour in Blue_Contours:
                    if cv2.contourArea(contour) > self.MIN_CONTOUR_AREA_THRESHOLD:
                        cv2.drawContours(frame, [contour], -1, self.contour_colors.get('blue'), 2)
                        Shelves += 1
                        
                        M = cv2.moments(contour)
                        if M["m00"] != 0:
                            cX = int(M["m10"] / M["m00"])
                            cY = int(M["m01"] / M["m00"])
                            cX = frame.shape[1] // 2 - cX
                        # X,Y Coords
                            center_points.append((cX, cY))
                        # Bearing
                            bearing.append(cX)
                
                if len(bearing) == 2:
                    if bearing[0] > bearing[1]:
                        Diff = bearing[0] - bearing[1]
                        CenDiff = Diff/2
                        CenBearing = bearing[1]+CenDiff
                    elif bearing[1] > bearing[0]:
                        Diff = bearing[1] - bearing[0]
                        CenDiff = Diff/2
                        CenBearing = bearing[0]+CenDiff
                    
                #View
                cv2.imshow('Camera Feed', Blue_frame)
                cv2.imshow('Masked View', Blue_Mask)

                # Check for the 'q' key to exit the loop
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

                if (Shelves == 1):
                    print("Shelve1 [", bearing[0], "]")
                    return True, bearing
                elif (Shelves == 2):
                    print("Shelve2 [", CenBearing, "]")
                    return True
                elif (Shelves > 2):
                    print("error")
                    return False
                else:    
                    print("clear")
                    return True            
        print("error")
        return False
    
    def PackZone(self):
        while True:
            ret, frame = self.cap.read()
            if not ret:
                break
            Yellow_Contours, Yellow_Mask, Yellow_frame = self.detect_color_objects(frame, self.color_ranges.get('yellow'))
            Zone_detected = 0
            center_points = []
            bearing = []
            for contour in Yellow_Contours:
                if cv2.contourArea(contour) > self.MIN_CONTOUR_AREA_THRESHOLD:
                    cv2.drawContours(frame, [contour], -1, self.contour_colors.get('yellow'), 2)
                    Zone_detected += 1
                    # X,Y Coords
                    M = cv2.moments(contour)
                    if M["m00"] != 0:
                        cX = int(M["m10"] / M["m00"])
                        cY = int(M["m01"] / M["m00"])
                        cX = frame.shape[1] // 2 - cX
                        center_points.append((cX, cY))
                        bearing.append(cX)
                    # Bearing
            #View
            cv2.imshow('Camera Feed', Yellow_frame)
            cv2.imshow('Masked View', Yellow_Mask)

             # Check for the 'q' key to exit the loop
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            if (Zone_detected == 1):
                print("Zone Detected [", bearing[0], "]")
                return True, bearing
            elif (Zone_detected > 1):
                print("Too many Zones/error")
                return False
            else:    
                print("No Zone")
                return False            
        print("error")
        return False
    
    # def Object(self):
    #     return True





if __name__ == "__main__":
    vision = Vision() 
    vision.distance()   
    vision.display()
    while True:
        vision.PackZone()

   # vision.release_camera()

