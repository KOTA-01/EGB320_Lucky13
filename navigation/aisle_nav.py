import time
import cv2
import numpy as np
import math
from itemIndex import item_to_index
from Read_order_class import OrderReader
order_reader = OrderReader()
from bayDistanceIndex import distance_from_wall
from enum import IntEnum
from rowdetection import RowMarkerDetector
rowdetect = RowMarkerDetector()
from shelf_aisle_index import WarehouseLayout
layout = WarehouseLayout()
from orient_avoidance import ObstacleDetectedException
from mobility.Motor_init import DFRobot_DC_Motor
from mobility.Motor_init import DFRobot_DC_Motor_IIC
from mobility.motor_control import stop
from mobility.motor_control import Motor
from mobility.motor_control import turn
from mobility.motor_control import turn_indefinitely
from mobility.motor_control import steering
from ultrasonic import UltrasonicSensor
distance = UltrasonicSensor()

#change
class robot(object):

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
						print("initialising centre")
						initial_bearing = rowdetect.get_detected_row_marker_bearing()
						proximity = distance.get_distance() # Get the ranges from the ultrasonic sensor
						closest_shelf = self.getclosestshelf()

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
						row_marker_bearing = rowdetect.get_detected_row_marker_bearing()
						current_range = rowdetect.get_detected_row_marker_range()
						proximity = distance.get_distance() # Get the ranges from the ultrasonic sensor

						if proximity < 0.2:
							print("repositioning for entry")
							state = reposition      
							break   

						else:
							state = drive           

				elif state == drive:
						# Adjust orientation based on bearing
						if row_marker_bearing and abs(row_marker_bearing) > angular_tolerance:
							angular_velocity = -0.05 if row_marker_bearing < 0 else 0.05 # Find the average velocity for theta
						else:
							angular_velocity = 0 # Find the motions needed stop

						# If we are within the acceptable range of the bay, stop
						if target_distance - linear_tolerance <= current_range <= target_distance + linear_tolerance:
							steering(0, 0) # Find the motions needed stop
							state = complete
							break

						# If we pass the bay (i.e., too close to the end marker), reverse
						elif current_range < target_distance - linear_tolerance:
							linear_velocity = -0.08 # Find the motions needed backwards slow

						# Otherwise, drive forward towards the bay
						else:
							linear_velocity = 0.08 # Find the motions needed forward slow
						
						steering(linear_velocity, angular_velocity)
						time.sleep(0.1)
				
				elif state == reposition:
					print(f"previous aisle: {self.previous_aisle}")

					while True:
						proximity = distance.get_distance() # Get the ranges from the ultrasonic sensor
						closest_shelf = self.getClosestShelf() # Using this to see if there's a shelf
						current_aisle = self.updatecurrentAisle()
						rowMaker = rowdetect.GetDetectedRowMarker()
						if closest_shelf:
							if proximity <= 0.5:
								print("In reposition state: avoiding obstacle...")
								if current_aisle < self.previous_aisle:
									proximity = distance.get_distance() # Get the ranges from the ultrasonic sensor
									current_aisle = self.updatecurrentAisle()
									rowMaker = rowdetect.GetDetectedRowMarker()
									Motor("RotateL_90") # 90 degrees Left
									steering(0.08, 0)
									time.sleep(2) # Find the motions needed forwards slow
									turn_indefinitely("Right")

									if rowMaker:
										stop() 
										state = initialise
										break
												

								elif current_aisle > self.previous_aisle:
									proximity = distance.get_distance() # Get the ranges from the ultrasonic sensor
									current_aisle = self.updatecurrentAisle()
									rowMaker = rowdetect.GetDetectedRowMarker()
									Motor("RotateR_90") # 90 degrees Right
									steering(0.08, 0)
									time.sleep(2) # Find the motions needed forwards slow
									turn_indefinitely("Left")
									if rowMaker:
										stop  # Find the motions needed stop
										state = initialise
										break
								
								elif current_aisle == self.previous_aisle:
									while proximity < 0.5:
										proximity = distance.get_distance()
										current_aisle = self.updatecurrentAisle()
										rowMaker = rowdetect.GetDetectedRowMarker()
										Motor("Backward_40")
										if proximity >= 0.5:
											state = initialise
											break
												
								else:
									state = initialise
									break

				elif state == complete:
					print("I'm at the bay")
					time.sleep(2)
					break

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
				row_marker_range = rowdetect.get_detected_row_marker_range()
				if row_marker_range is not None:
					range_init = rowdetect.get_detected_row_marker_range()
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
					
				print("YELLOW LED - Picking up item ...")
				time.sleep(5) # Picking the item (Primo adjusts this)
				state = done
			
			elif state == done:
				print("item picked up!")
				break


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
						rotation_angle += 0.1 * 0.3
						detectedRowMarker = rowdetect.GetDetectedRowMarker()
						proximity = distance.get_distance() # Get the ranges from the ultrasonic sensor
						
						if detectedRowMarker:
							stop() # Find the motions to stop
							state = identify_destination
							break

						if proximity < 0.2:
							stop() # Find the motions to stop
							print("Possible obstacle, moving away")
							state = orient_obstacle_avoidance
							raise ObstacleDetectedException

				except ObstacleDetectedException:
					pass
					
				if not detectedRowMarker and state != orient_obstacle_avoidance:
					state = reposition
				
			elif state == orient_obstacle_avoidance:
				closest_shelf = self.getClosestShelf() # Find this later
				if closest_shelf:
					while proximity <= 0.2:
						print("In orient obstacle avoidance state...")
						Motor("Backward_40") # Find the motions to slowly reverse
						time.sleep(0.1)  # Assume a 100 ms sleep duration
						proximity = distance.get_distance()  # Update proximity
						print(f"Current proximity: {proximity}")
					state = orient 
				
				else:
					state = orient
			
			elif state == reposition:
				stop() # Stop
				print("unable to find row marker")
				time.sleep(2)
				turn_indefinitely("Right") # Rotate 360 degrees
				self.aligning()

				proximity = distance.get_distance() # Get the ranges from the ultrasonic sensor
				if proximity < 0.2:
					state = avoid_obstacle_unseen_marker

				Motor("Forward_60") # Drive forwards
				time.sleep(1)
				state = orient


			elif state == identify_destination:
				self.initAisle()
				rowdetect.GetDetectedRowMarker()
				current_aisle = self.updatecurrentAisle()
				if not hasattr(self, 'previous_aisle'):
					self.previous_aisle = current_aisle
					print(f"previous aisle: {self.previous_aisle}")
				proximity = distance.get_distance() # Get the ranges from the ultrasonic sensor

				if proximity < 0.3:
					state = avoid_obstacle_seen_marker
					return self.previous_aisle

				elif current_aisle < aisle:
					print("The aisle destination is to the right of me")
					Motor("RotateR_90") # Rotate 90 degrees to the right

					start_time = time.time()
					while time.time() - start_time < 3:
						proximity = distance.get_distance() # Get the ranges from the ultrasonic sensor
						if proximity < 0.1:
							print("Obstacle detected whilst turning")
							stop() # Stop
							state = avoid_obstacle_seen_marker
							break
						time.sleep(0.1)

					if state != avoid_obstacle_seen_marker:
						Motor("Forwards_80") # Drive forwards
						start_time = time.time()
						while time.time() - start_time < 3:
							proximity = distance.get_distance() # Get the ranges from the ultrasonic sensor
							if proximity < 0.1:
								print("Obstacle detected while moving forward!")
								stop() # Stop
								state = avoid_obstacle_seen_marker
								break
							time.sleep(0.1)

				elif current_aisle > aisle:
					print("The aisle destination is to the left of me")
					Motor("RotateL_90") # Rotate 90 degrees to the left

					start_time = time.time()
					while time.time() - start_time < 3:
						proximity = distance.get_distance() # Get the ranges from the ultrasonic sensor
						if proximity < 0.3:
							print("Obstacle detected whilst turning")
							stop() # Stop
							state = avoid_obstacle_seen_marker
							break
						time.sleep(0.1)

					if state != avoid_obstacle_seen_marker:
						Motor("Forwards_80") # Drive forwards
						start_time = time.time()
						while time.time() - start_time < 3:
							proximity = distance.get_distance() # Get the ranges from the ultrasonic sensor
							if proximity < 0.3:
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
				detected_shelf = self.getClosestShelf() # Find this later
				
				Motor("RotateR_30")
				Motor("Backward_40") # Drive backwards
				time.sleep(2)

				proximity = distance.get_distance() # Get the ranges from the ultrasonic sensor
				if proximity >= 0.2:
					print("Safe distance, looking for row marker")
					state = orient

				else:
					state = avoid_obstacle_unseen_marker
			
			elif state == avoid_obstacle_seen_marker:
				print("Object in the way, navigating around...")
				stop() # Stop
				detected_shelf = self.getClosestShelf() # Find this later
				proximity = distance.get_distance() # Get the ranges from the ultrasonic sensor
				
				if detected_shelf:
					if current_aisle < aisle:
						Motor("RotateL_30")
						Motor("Forward_40")
						time.sleep(2)
					elif current_aisle > aisle:
						Motor("RotateR_30")
						Motor("Forward_40")
						time.sleep(2)
				
				if proximity < 0.3:
					while proximity < 0.3:
						print("In obstacle avoidance state...")
						Motor("Backward_40") # Drive backwards
						time.sleep(0.1)  # Assume a 100 ms sleep duration
						proximity = distance.get_distance()  # Update proximity
						print(f"Current proximity: {proximity}")
					state = orient 


				proximity = distance.get_distance() # Get the ranges from the ultrasonic sensor
				if proximity >= 0.3:
					print("Safe distance, looking for row marker")
					Motor("Forward_40") # Drive forwards
					time.sleep(3)
					state = identify_destination

				else:
					state = avoid_obstacle_seen_marker
			
			if state == done:
				break

	def exiting(self):
		current_order = order_reader.ReadOrder("Order_1.csv")
		shelf_number = int(current_order["shelf"])
		if shelf_number % 2 == 0: # Even number shelf
			angular_velocity = "RotateR_90" # Rotate right
		else: # odd number shelf
			angular_velocity = "RotateL_90" # Rotate left

		while True:
			initial_bearing = rowdetect.get_detected_row_marker_bearing()

			if initial_bearing is not None:
				break

			steering(0, angular_velocity)
			time.sleep(0.1)

		stop() # Stop
		self.exitnav()

	def aligning(self):
		# Stage 1: Locate the Packing Bay
		while True:
			packing_bay_rb = self.GetDetectedPackingBay()
			if packing_bay_rb:
				stop()  # Stop rotating
				break
			else:
				turn_indefinitely("Left")  # Rotate until detected
			time.sleep(0.1)

		# Stage 2: Align with Rightmost Edge
		while True:
			packing_bay_rb = self.GetDetectedPackingBay()
			if not packing_bay_rb:
				stop()  # Stop rotating once out of view
				break
			else:
				turn_indefinitely("Right")  # Rotate in the opposite direction
			time.sleep(0.1)
					

	def navigate_to_packing_bay(self):
		angular_tolerance = 0.1
		safe_stopping_distance = 0.02
		# Stage 1: Rotate until the packing bay is centered
		while True:

			packing_bay_rb = self.GetDetectedPackingBay()
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
			proximity = distance.get_distance()
			packing_bay_rb = self.GetDetectedPackingBay()
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
			initial_bearing = self.GetDetectedRowMarker_bearing()
			
			if not initial_bearing or abs(initial_bearing) < angular_tolerance:
				break
			
			angular_velocity = -0.05 if initial_bearing < 0 else 0.05
			steering(0, angular_velocity)
			time.sleep(0.1)

		# 3. Drive and Adjust based on Row Marker's bearing and distance
		while True:
			row_marker_bearing = self.GetDetectedRowMarker_bearing()
			current_range = distance.get_distance() # connect to the ultrasonic

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
			blob_info = rowdetect.vision.find_information()

			if blob_info is not None:
				blob_count = blob_info["blob_count"]

				# Check blob_count to determine current aisle
				if blob_count in [1, 2, 3]:  # Assuming aisles are represented by 1, 2, or 3 blobs respectively
					stop()
					self.currentAisle = blob_count - 1  # 0-indexed aisle number
					print(f"I am in aisle {self.currentAisle}")
					return  # Exit the function

			time.sleep(0.1)


	