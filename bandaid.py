import time

import cv2

import numpy as np

import math

from itemIndex import item_to_index

from Read_order_class import OrderReader

from bayDistanceIndex import distance_from_wall

from enum import IntEnum

from shelf_aisle_index import WarehouseLayout

from orient_avoidance import ObstacleDetectedException

from Motor_Task import stop

from Motor_Task import Motor

from Motor_Task import turn_indefinitely

from Motor_Task import steering

from ultrasonic import GroveUltrasonicRanger

from vision_module.Vision import Vision

#from item_collection.collection01 import Collection

#collect = Collection()

ultra = GroveUltrasonicRanger(26)

layout = WarehouseLayout()

order_reader = OrderReader()

vision = Vision()



#change

class robot(object):

    def __init__ (self):

        pass



    def degrees_to_radians(degrees):

        return degrees * (math.pi/180)

    

    def nav_to_aisle(self):

        orient = 1

        reposition = 2

        identify_destination = 3

        avoid_obstacle_unseen_marker = 4

        avoid_obstacle_seen_marker = 5

        orient_obstacle_avoidance = 6

        done = 7



        state = orient



        current_order = order_reader.ReadOrder("Order_1.csv")

        shelf_number = current_order["shelf"]

        aisle = layout.shelf_to_aisle[shelf_number]

        rotation_angle = 0



        while True:

            if state == orient:

                rotation_angle = 0

                turn_indefinitely("Left") # rotate 360 degrees

                try: 

                    while rotation_angle < 2 * math.pi:

                        data = vision.find_information()

                        for info in data:

                            color_name, bearing, distance = info

                            if color_name == 'black':

                                detectedRowMarker = vision.detected_objects_count('black')

                        rotation_angle += 0.1 * 0.3

                        proximity = ultra.get_distance() # Get the ranges from the ultrasonic sensor

                        

                        if detectedRowMarker:

                            stop() # Find the motions to stop

                            state = identify_destination

                            break



                        if proximity < 0.2:

                            stop() # Find the motions to stop

                            print("Possible obstacle, moving away")

                            state = orient_obstacle_avoidance

                            raise ObstacleDetectedException

                        

                        time.sleep(0.1)



                except ObstacleDetectedException:

                    pass

                    

                if not detectedRowMarker and state != orient_obstacle_avoidance:

                    state = reposition

                

            elif state == orient_obstacle_avoidance:

                data = vision.find_information()

                for info in data:

                    color_data = {color_name: (float(bearing), float(distance)) for color_name, bearing, distance in data}



                    if 'green' in color_data and color_data['green'][1] < 0.2:

                        obstacle_bearing = color_data['green'][0]



                    elif 'blue' in color_data and proximity < 0.2:

                        closest_shelf = vision.detected_objects_count('blue')

                    

                if closest_shelf:

                    Motor("RotateR_180")

                    if 'green' in color_data and color_data['green'][1] < 0.2:

                        obstacle_bearing = color_data['green'][0]

                        time.sleep(1)

                        if obstacle_bearing < 0:

                            turn_indefinitely("Right")

                        else:

                            turn_indefinitely("Left")

                        

                        Motor("Forward_40")

                        time.sleep(1)

                        state = orient



                    else:

                        Motor("Forward_40")

                        time.sleep(3)

                        stop()

                        state = orient 

                

                else:

                    state = orient

            

            elif state == reposition:

                stop() # Stop

                print("unable to find row marker")

                time.sleep(2)

                turn_indefinitely("Right") # Rotate 360 degrees

                self.aligning()





                data = vision.find_information()

                for info in data:

                    color_data = {color_name: (float(bearing), float(distance)) for color_name, bearing, distance in data}



                    if 'green' in color_data and color_data['green'][1] < 0.2:

                        state = avoid_obstacle_unseen_marker



                proximity = ultra.get_distance() # Get the ranges from the ultrasonic sensor

                if proximity < 0.2:

                    state = avoid_obstacle_unseen_marker



                Motor("Forward_60") # Drive forwards

                time.sleep(2)

                state = orient





            elif state == identify_destination:

                self.initAisle()

                vision.detected_objects_count('black')

                current_aisle = self.updatecurrentAisle()

                if not hasattr(self, 'previous_aisle'):

                    self.previous_aisle = current_aisle

                    print(f"previous aisle: {self.previous_aisle}")

                proximity = ultra.get_distance() # Get the ranges from the ultrasonic sensor



                if proximity < 0.3:

                    state = avoid_obstacle_seen_marker

                    return self.previous_aisle



                elif current_aisle < aisle:

                    print("The aisle destination is to the right of me")

                    Motor("RotateR_90") # Rotate 90 degrees to the right



                    Motor("Forwards_60") # Drive forwards

                    start_time = time.time()

                    while time.time() - start_time < 3:

                        data = vision.find_information()

                        proximity = ultra.get_distance() # Get the ranges from the ultrasonic sensor

                        for info in data:

                            color_data = {color_name: (float(bearing), float(distance)) for color_name, bearing, distance in data}

                            if 'green' in color_data and color_data['green'][1] < 0.2:

                                state = avoid_obstacle_seen_marker



                        if proximity < 0.2:

                            print("Obstacle detected while moving forward!")

                            stop() # Stop

                            state = avoid_obstacle_seen_marker

                            break



                        time.sleep(0.1)



                elif current_aisle > aisle:

                    print("The aisle destination is to the left of me")

                    Motor("RotateL_90") # Rotate 90 degrees to the left



                    Motor("Forwards_60") # Drive forwards

                    start_time = time.time()

                    while time.time() - start_time < 3:

                        data = vision.find_information()

                        proximity = ultra.get_distance() # Get the ranges from the ultrasonic sensor

                        for info in data:

                            color_data = {color_name: (float(bearing), float(distance)) for color_name, bearing, distance in data}

                            if 'green' in color_data and color_data['green'][1] < 0.2:

                                state = avoid_obstacle_seen_marker



                        if proximity < 0.2:

                            print("Obstacle detected while moving forward!")

                            stop() # Stop

                            state = avoid_obstacle_seen_marker

                            break



                        time.sleep(0.1)

                        

                elif current_aisle == aisle:

                    print("I'm in the right aisle!")

                    state = done

                    break



                else:

                    print("Can't find row marker, orientating myself...")

                    state = orient



            elif state == avoid_obstacle_unseen_marker:

                print("Object in the way, navigating around...")

                stop() # Stop

                while True:

                    proximity = ultra.get_distance()

                    data = vision.find_information()



                    color_data = {color_name: (float(bearing), float(distance)) for color_name, bearing, distance in data}



                    if 'green' in color_data and color_data['green'][1] < 0.2:

                        obstacle_bearing = color_data['green'][0]



                        # Obstacle avoidance

                        Motor("Backward_40")

                        time.sleep(1)

                        if obstacle_bearing < 0:

                            turn_indefinitely("Right")

                        else:

                            turn_indefinitely("Left")

                        

                        Motor("Forward_40")

                        time.sleep(1)

                    

                    elif 'blue' in color_data:

                        turn_indefinitely("Right")

                        time.sleep(0.1)



                    else:

                        print("Safe to move, transitioning to orientation mode.")

                        state = orient

            

            elif state == avoid_obstacle_seen_marker:

                print("Object in the way, navigating around...")

                stop() # Stop

                detected_shelf = vision.detected_objects_count('blue')

                proximity = ultra.get_distance() # Get the ranges from the ultrasonic sensor

                

                if detected_shelf:

                    target_direction = "RotateL_30" if current_aisle < aisle else "RotateR_30"

                    Motor(target_direction)

                    Motor("Forward_40")

                    time.sleep(2)

                

                if proximity < 0.3:

                    while proximity < 0.3:

                        print("In obstacle avoidance state...")

                        Motor("Backward_40") # Drive backwards

                        time.sleep(0.1)  # Assume a 100 ms sleep duration

                        proximity = ultra.get_distance()  # Update proximity

                        print(f"Current proximity: {proximity}")

                    state = orient 





                proximity = ultra.get_distance() # Get the ranges from the ultrasonic sensor

                if proximity >= 0.3:

                    print("Safe distance, looking for row marker")

                    Motor("Forward_40") # Drive forwards

                    time.sleep(3)

                    state = identify_destination



                else:

                    state = avoid_obstacle_seen_marker

            

            if state == done:

                break





    def bayNav(self, bay_number):

            target_distance = distance_from_wall(bay_number)

            linear_tolerance = 0.02

            angular_tolerance = 0.05

            lateral_tolerance = 0.05

            time.sleep(2)



            initialise = 1

            reposition = 2

            adjust = 3

            drive = 4

            complete = 5

            

            state = initialise



            while True:

                if state == initialise:

                    # 2. Adjusting bearing towards the end row marker

                    while True:

                        data = vision.find_information()

                        for info in data:

                            color_name, bearing, distance = info

                            if color_name == 'black':

                                initial_bearing = self.degrees_to_radians(float(bearing))  # converting to float before converting to radians

                                print(f"Bearing for black object: {initial_bearing} radians")

                            if color_name == 'blue':

                                closest_shelf = vision.detected_objects_count('blue')

                                

                        print("initialising centre")

                        proximity = ultra.get_distance() # Get the ranges from the ultrasonic sensor

                        

                        if proximity < 0.2:

                            if closest_shelf:

                                print("repositioning to get a better entry")

                                stop() # Stop

                                state = reposition

                                break



                        if initial_bearing is None:

                            print("Marker not detected, searching...")

                            stop()

                            state = reposition

                            break

                            

                        if not initial_bearing or abs(initial_bearing) < angular_tolerance:

                            state = drive

                            break

                        else:

                            angular_velocity = -0.05 if initial_bearing < 0 else 0.05

                            steering(0, angular_velocity) # Find the motions needed

                            time.sleep(0.1)





                elif state == adjust:

                    # 3. Drive and Adjust based on Row Marker's bearing and distance

                    while True:

                        data = vision.find_information()

                        for info in data:

                            color_name, bearing, distance = info

                            if color_name == 'black':

                                row_marker_bearing = self.degrees_to_radians(float(bearing))  # converting to float before converting to radians

                                print(f"Bearing for black object: {initial_bearing} radians")

                        proximity = ultra.get_distance() # Get the ranges from the ultrasonic sensor



                        if proximity < 0.2:

                            print("repositioning for entry")

                            state = reposition      

                            break   



                        else:

                            state = drive           



                elif state == drive:

                    while True:

                        data = vision.find_information()

                        for info in data:

                            color_name, bearing, distance = info

                            if color_name == 'black':

                                row_marker_bearing = self.degrees_to_radians(float(bearing))  # converting to float before converting to radians

                                print(f"Bearing for black object: {initial_bearing} radians")

                        current_range = ultra.get_distance() # Get the ranges from the ultrasonic sensor

                        # Adjust orientation based on bearing

                        if row_marker_bearing and abs(row_marker_bearing) > angular_tolerance:

                            angular_velocity = -0.05 if row_marker_bearing < 0 else 0.05 # Find the average velocity for theta

                        else:

                            angular_velocity = 0 



                        # If we are within the acceptable range of the bay, stop

                        if target_distance - linear_tolerance <= current_range <= target_distance + linear_tolerance:

                            stop() # Find the motions needed stop

                            state = complete

                            break



                        # If we pass the bay (i.e., too close to the end marker), reverse

                        elif current_range < target_distance - linear_tolerance:

                            linear_velocity = -0.04 # Find the motions needed backwards slow



                        # Otherwise, drive forward towards the bay

                        else:

                            linear_velocity = 0.04 # Find the motions needed forward slow

                        

                        steering(linear_velocity, angular_velocity)

                        time.sleep(0.1)

                

                elif state == complete:

                    print("I'm at the bay")

                    time.sleep(2)

                    break



                elif state == reposition:

                    print(f"previous aisle: {self.previous_aisle}")



                    while True:

                        data = vision.find_information()

                        rowmarker, closest_shelf = self.get_bearing_and_shelf(data, 'black'), self.get_bearing_and_shelf(data, 'blue')[1]

                        proximity = ultra.get_distance



                        if rowmarker:

                            rowmarker = self.degrees_to_radians(rowmarker)

                            print(f"bearing for rowmaker: {rowmarker} radians")



                        proximity = ultra.get_distance()

                        current_aisle = self.updatecurrentAisle()



                        color_data = {color_name: (float(bearing), float(distance)) for color_name, bearing, distance in data}

                        if 'green' in color_data and color_data['green'][1] < 0.2:

                            obstacle_bearing = color_data['green'][0]

                            if obstacle_bearing < 0:

                                turn_indefinitely("Right")

                            else:

                                turn_indefinitely("Left")

                            

                            Motor("Forward_40")

                            time.sleep(1)

                        

                        if closest_shelf and proximity <= 0.5:

                            print("In reposition state: avoiding obstacle...")

                            if current_aisle < self.previous_aisle:

                                self.navigate("RotateL_90", "Right", rowmarker)

                            elif current_aisle > self.previous_aisle:

                                self.navigate("RotateR_90", "Left", rowmarker)

                            elif current_aisle == self.previous_aisle:

                                self.navigate_backward_until_clear(proximity)

                            else:

                                state = initialise

                        else:

                               state = initialise

                        

                    

    def run(self):

        orient = 1

        marker = 2

        nav_to_bay = 3

        at_bay = 4

        done = 5



        state = orient	

        while True:

            if state == orient:

                self.initAisle()

                self.updatecurrentAisle()

                print("Driving down the aisle")

                state = nav_to_bay



            elif state == marker:

                data = vision.find_infomation()

                for info in data:

                    color_name, bearing, distance = info

                    if color_name == 'black':

                        rowmarker = distance



                row_marker_range_camera = vision.detected_objects_count('black')

                if row_marker_range_camera is not None:

                    range_init = ultra.get_distance

                    print("The range is: %0.4f" %(range_init))





                    state = nav_to_bay



            elif state == nav_to_bay:

                try:

                    current_order = order_reader.ReadOrder("Order_1.csv")

                    if current_order:

                        bay_number = int(current_order["bay"])

                        print("Go to bay: %0.4f" %(bay_number))

                        time.sleep(2)

                        self.bayNav(bay_number)

                    

                        state = at_bay

                    else:

                        print("Order not found tehehe")



                except Exception as e:

                    print(f"Error reading order: {e}")

                    state = marker



            elif state == at_bay:

                shelf_number = int(current_order["shelf"])

                print("Go to shelf: %0.4f" %(shelf_number))



                if shelf_number % 2 == 0: # Even number shelf so left

                    Motor("RotateL_90")

                else: # odd number shelf so right

                    Motor("RotateR_90")



                shelfheight = int(current_order["height"])

                print("Go to shelf: %0.4f" %(shelfheight))    

                    

                print("YELLOW LED - Picking up item ...")

                time.sleep(5)

                state = done

            

            elif state == done:

                print("item picked up!")

                break





    def exiting(self):

        current_order = order_reader.ReadOrder("Order_1.csv")

        shelf_number = int(current_order["shelf"])

        if shelf_number % 2 == 0: # Even number shelf

            angular_velocity = "RotateR_90" # Rotate right

        else: # odd number shelf

            angular_velocity = "RotateL_90" # Rotate left



        while True:

            data = vision.find_information()

            for info in data:

                color_name, bearing, distance = info

                if color_name == 'black':

                    initial_bearing = self.degrees_to_radians(float(bearing))  # converting to float before converting to radians

                    print(f"Bearing for black object: {initial_bearing} radians")



            if initial_bearing is not None:

                break



            steering(0, angular_velocity)

            time.sleep(0.1)



        stop() # Stop

        self.exitnav()



    def aligning(self):

        # Stage 1: Locate the Packing Bay

        while True:

            data = vision.find_information

            for info in data:

                color_name, bearing, distance = info

                if color_name == 'yellow':

                    packing_bay_rb = self.degrees_to_radians(float(bearing))  # converting to float before converting to radians

                    print(f"Bearing for yellow object: {packing_bay_rb} radians")

            if packing_bay_rb:

                stop()  # Stop rotating

                break

            else:

                turn_indefinitely("Left")  # Rotate until detected

            time.sleep(0.1)



        # Stage 2: Align with Rightmost Edge

        while True:

            data = vision.find_information

            for info in data:

                color_name, bearing, distance = info

                if color_name == 'yellow':

                    packing_bay_rb = self.degrees_to_radians(float(bearing))  # converting to float before converting to radians

                    print(f"Bearing for yellow object: {packing_bay_rb} radians")

            if not packing_bay_rb:

                stop()  # Stop rotating once out of view

                break

            else:

                turn_indefinitely("Right")  # Rotate in the opposite direction

            time.sleep(0.1)

                    



    def navigate_to_packing_bay(self):

        angular_tolerance = 0.1

        safe_stopping_distance = 0.03

        # Stage 1: Rotate until the packing bay is centered

        while True:



            data = vision.find_information

            for info in data:

                color_name, bearing, distance = info

                if color_name == 'yellow':

                    packing_bay_rb = self.degrees_to_radians(float(bearing))  # converting to float before converting to radians

                    print(f"Bearing for yellow object: {packing_bay_rb} radians")

            if not packing_bay_rb:

                turn_indefinitely("Right")  # Rotate if the packing bay is not in view

                continue



            _range, _bearing = packing_bay_rb



            print(f"Bearing: {_bearing}")

            

            # If bearing is small enough (packing bay is centered), break to move to Stage 2

            if abs(_bearing) < angular_tolerance:  # Assume 0.05 rad is sufficiently centered

                stop()  # Stop rotating

                break



            angular_velocity = -0.05 if _bearing < 0 else 0.05

            steering(0, angular_velocity)

            time.sleep(0.1)



        # Stage 2: Drive straight towards the packing bay

        while True:

            proximity = ultra.get_distance()

            packing_bay_rb = vision.detected_objects_count('yellow')

            if not packing_bay_rb:

                print("Lost sight of the packing bay while driving towards it!")

                break  # Or implement behavior to re-orient towards the packing bay



            _range, _bearing = packing_bay_rb

            

            # If we're close enough, stop

            if proximity < safe_stopping_distance:  # Assume 0.5 m is a safe stopping distance

                stop

                print("Reached the packing bay")

                break



            # Move towards the packing bay

            Motor("Forward_60")  # Drive straight forward

            time.sleep(0.1)



    def exitnav(self):

        target_distance = 1.1  # Distance you want to be from the row marker

        linear_tolerance = 0.02

        angular_tolerance = 0.05

        lateral_tolerance = 0.05  # Note: You don't use this variable in the provided code

        time.sleep(2)



        while True:

            data = vision.find_information()

            for info in data:

                color_name, bearing, distance = info

                if color_name == 'black':

                    initial_bearing = self.degrees_to_radians(float(bearing))  # converting to float before converting to radians

                    print(f"Bearing for black object: {initial_bearing} radians")

            

            if not initial_bearing or abs(initial_bearing) < angular_tolerance:

                break

            

            angular_velocity = -0.05 if initial_bearing < 0 else 0.05

            steering(0, angular_velocity)

            time.sleep(0.1)



        # 3. Drive and Adjust based on Row Marker's bearing and distance

        while True:

            data = vision.find_information()

            for info in data:

                color_name, bearing, distance = info

                if color_name == 'black':

                    row_marker_bearing = self.degrees_to_radians(float(bearing))  # converting to float before converting to radians

                    print(f"Bearing for black object: {initial_bearing} radians")

            current_range = ultra.get_distance() # connect to the ultrasonic



            # Adjust orientation based on bearing

            if row_marker_bearing and abs(row_marker_bearing) > angular_tolerance:

                angular_velocity = -0.05 if row_marker_bearing < 0 else 0.05

            else:

                angular_velocity = 0



            # If we are within the acceptable range of the bay, stop

            if abs(current_range - target_distance) <= linear_tolerance:

                stop()

                break



            # If we are further than the target distance from the row marker, move backward

            elif current_range < target_distance:

                linear_velocity = -0.05  # Negative to move backward



            # Otherwise, stop (should not really get here with the above conditions, but just in case)

            else:

                linear_velocity = 0

            

            steering(linear_velocity, angular_velocity)

            time.sleep(0.1)



    def initAisle(self):

        self.currentAisle = -1



    def updatecurrentAisle(self):

        turn_indefinitely("Left") # rotate 360 degrees until marker is seen

        

        while True:

            # Get blob info from the Vision class

            data = vision.find_information()

            for info in data:

                color_name, bearing, distance, aisle = info

                if color_name == 'black':

                    i = aisle

                    print(f"Bearing for black object: {aisle} radians")

                    return i



            time.sleep(0.1)



    def get_bearing_and_shelf(self, data, color_target):

        for color_name, bearing, _ in data:

            if color_name == color_target:

                return float(bearing), vision.detected_objects_count(color_target)

        return None, None



    def navigate(turn_direction, after_turn_direction, rowmarker):

        Motor(turn_direction)

        steering(0.08, 0)

        time.sleep(2)

        turn_indefinitely(after_turn_direction)



            

    def navigate_backward_until_clear(proximity):

        while proximity < 0.5:

            proximity = ultra.get_distance()

            Motor("Backward_40")

            if proximity >= 0.5:

                return 'initialise'

    

