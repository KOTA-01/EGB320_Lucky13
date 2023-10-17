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
						proximity = self.readProximity() # Get the ranges from the ultrasonic sensor

						if proximity < 0.5:
							print("repositioning to get a better entry")
							self.SetTargetVelocities(0, 0) # Find the motions needed
							state = reposition
							break
							
						if not initial_bearing or abs(initial_bearing) < angular_tolerance:
							state = drive
							break
						else:
							angular_velocity = -0.05 if initial_bearing < 0 else 0.05
							self.SetTargetVelocities(0, angular_velocity) # Find the motions needed
							time.sleep(0.1)


				elif state == adjust:
					# 3. Drive and Adjust based on Row Marker's bearing and distance
					while True:
						row_marker_bearing = rowdetect.get_detected_row_marker_bearing()
						current_range = rowdetect.get_detected_row_marker_range()
						proximity = self.readProximity() # Get the ranges from the ultrasonic sensor

						if proximity < 0.5:
							print("repositioning for entry")
							state = reposition      
							break   

						else:
							state = drive           

				elif state == drive:
						# Adjust orientation based on bearing
						if row_marker_bearing and abs(row_marker_bearing) > angular_tolerance:
							angular_velocity = -0.05 if row_marker_bearing < 0 else 0.05 # Find the motions needed
						else:
							angular_velocity = 0 # Find the motions needed stop

						# If we are within the acceptable range of the bay, stop
						if target_distance - linear_tolerance <= current_range <= target_distance + linear_tolerance:
							self.SetTargetVelocities(0, 0) # Find the motions needed stop
							state = complete
							break

						# If we pass the bay (i.e., too close to the end marker), reverse
						elif current_range < target_distance - linear_tolerance:
							linear_velocity = -0.05 # Find the motions needed backwards slow

						# Otherwise, drive forward towards the bay
						else:
							linear_velocity = 0.05 # Find the motions needed forward slow
						
						self.SetTargetVelocities(linear_velocity, angular_velocity)
						time.sleep(0.1)
				
				elif state == reposition:
					print(f"previous aisle: {self.previous_aisle}")

					while True:
						proximity = self.readProximity() # Get the ranges from the ultrasonic sensor
						closest_shelf = self.getClosestShelf() # Find this later
						current_aisle = self.updatecurrentAisle()
						rowMaker = rowdetect.GetDetectedRowMarker()
						if closest_shelf:
							if proximity <= 0.5:
								print("In reposition state: avoiding obstacle...")
								if current_aisle < self.previous_aisle:
									proximity = self.readProximity() # Get the ranges from the ultrasonic sensor
									current_aisle = self.updatecurrentAisle()
									rowMaker = rowdetect.GetDetectedRowMarker()
									self.SetTargetVelocities(0, 0.25)
									time.sleep(2) # Find the motions needed 90 degrees
									self.SetTargetVelocities(0.015, 0)
									time.sleep(2) # Find the motions needed forwards slow
									self.SetTargetVelocities(0, -0.25)
									time.sleep(2) # Find the motions needed 90 degrees

									if rowMaker:
										self.SetTargetVelocities(0, 0) # Find the motions needed
										state = initialise
										break
												

								elif current_aisle > self.previous_aisle:
									proximity = self.readProximity() # Get the ranges from the ultrasonic sensor
									current_aisle = self.updatecurrentAisle()
									rowMaker = rowdetect.GetDetectedRowMarker()
									self.SetTargetVelocities(0, -0.25)
									time.sleep(2) # Find the motions needed 90 degrees
									self.SetTargetVelocities(0.015, 0)
									time.sleep(2) # Find the motions needed forwards slow
									self.SetTargetVelocities(0, 0.25)
									time.sleep(2) # Find the motions needed 90 degrees
									if rowMaker:
										self.SetTargetVelocities(0, 0)  # Find the motions needed stop
										state = initialise
										break
								
								elif current_aisle == self.previous_aisle:
									while proximity < 0.5:
										proximity = self.readProximity()
										current_aisle = self.updatecurrentAisle()
										rowMaker = rowdetect.GetDetectedRowMarker()
										self.SetTargetVelocities(-0.02, 0) # Find the motions needed stop
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

				if shelf_number % 2 == 0: # Even number shelf
					angular_velocity = 0.15
					self.SetTargetVelocities(0, angular_velocity) # Find the motions needed 90 degrees
					time.sleep(4)
					self.SetTargetVelocities(0,0)
				else: # odd number shelf
					angular_velocity = -0.15
					self.SetTargetVelocities(0, angular_velocity) # Find the motions needed 90 degrees
					time.sleep(4)
					self.SetTargetVelocities(0,0)
					
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
				self.SetTargetVelocities(0, 0.3) # rotate 360 degrees
				try: 
					while rotation_angle < 2 * math.pi:
						rotation_angle += 0.1 * 0.3
						detectedRowMarker = rowdetect.GetDetectedRowMarker()
						proximity = self.readProximity() # Get the ranges from the ultrasonic sensor
						
						if detectedRowMarker:
							self.SetTargetVelocities(0, 0) # Find the motions to stop
							state = identify_destination
							break

						if proximity < 0.2:
							self.SetTargetVelocities(0, 0) # Find the motions to stop
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
						self.SetTargetVelocities(-0.02, 0) # Find the motions to slowly reverse
						time.sleep(0.1)  # Assume a 100 ms sleep duration
						proximity = self.readProximity()  # Update proximity
						print(f"Current proximity: {proximity}")
					state = orient 
				
				else:
					state = orient
			
			elif state == reposition:
				self.SetTargetVelocities(0, 0) # Stop
				print("unable to find row marker")
				time.sleep(2)
				self.SetTargetVelocities(0, 0.3) # Rotate 360 degrees
				self.aligning()

				proximity = self.readProximity() # Get the ranges from the ultrasonic sensor
				if proximity < 0.2:
					state = avoid_obstacle_unseen_marker

				self.SetTargetVelocities(0.05, 0) # Drive forwards
				time.sleep(1)
				state = orient


			elif state == identify_destination:
				self.initAisle()
				rowdetect.GetDetectedRowMarker()
				current_aisle = self.updatecurrentAisle()
				if not hasattr(self, 'previous_aisle'):
					self.previous_aisle = current_aisle
					print(f"previous aisle: {self.previous_aisle}")
				proximity = self.readProximity() # Get the ranges from the ultrasonic sensor

				if proximity < 0.3:
					state = avoid_obstacle_seen_marker
					return self.previous_aisle

				elif current_aisle < aisle:
					print("The aisle destination is to the right of me")
					self.SetTargetVelocities(0, -0.25) # Rotate 90 degrees to the right

					start_time = time.time()
					while time.time() - start_time < 3:
						proximity = self.readProximity() # Get the ranges from the ultrasonic sensor
						if proximity < 0.1:
							print("Obstacle detected whilst turning")
							self.SetTargetVelocities(0, 0) # Stop
							state = avoid_obstacle_seen_marker
							break
						time.sleep(0.1)

					if state != avoid_obstacle_seen_marker:
						self.SetTargetVelocities(0.05, 0) # Drive forwards
						start_time = time.time()
						while time.time() - start_time < 3:
							proximity = self.readProximity() # Get the ranges from the ultrasonic sensor
							if proximity < 0.1:
								print("Obstacle detected while moving forward!")
								self.SetTargetVelocities(0,0) # Stop
								state = avoid_obstacle_seen_marker
								break
							time.sleep(0.1)

				elif current_aisle > aisle:
					print("The aisle destination is to the left of me")
					self.SetTargetVelocities(0, 0.25) # Rotate 90 degrees to the left

					start_time = time.time()
					while time.time() - start_time < 3:
						proximity = self.readProximity() # Get the ranges from the ultrasonic sensor
						if proximity < 0.3:
							print("Obstacle detected whilst turning")
							self.SetTargetVelocities(0, 0) # Stop
							state = avoid_obstacle_seen_marker
							break
						time.sleep(0.1)

					if state != avoid_obstacle_seen_marker:
						self.SetTargetVelocities(0.05, 0) # Drive forwards
						start_time = time.time()
						while time.time() - start_time < 3:
							proximity = self.readProximity() # Get the ranges from the ultrasonic sensor
							if proximity < 0.3:
								print("Obstacle detected while moving forward!")
								self.SetTargetVelocities(0,0) # Stop
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
				self.SetTargetVelocities(0, 0) # Stop
				detected_shelf = self.getClosestShelf() # Find this later
				
				self.SetTargetVelocities(0, 0.2) # Rotate 30 - 45 degrees
				time.sleep(2)
				self.SetTargetVelocities(0.03, 0) # Drive forwards
				time.sleep(2)

				proximity = self.readProximity() # Get the ranges from the ultrasonic sensor
				if proximity >= 0.2:
					print("Safe distance, looking for row marker")
					state = orient

				else:
					state = avoid_obstacle_unseen_marker
			
			elif state == avoid_obstacle_seen_marker:
				print("Object in the way, navigating around...")
				self.SetTargetVelocities(0, 0) # Stop
				detected_shelf = self.getClosestShelf() # Find this later
				proximity = self.readProximity() # Get the ranges from the ultrasonic sensor
				
				if detected_shelf:
					if current_aisle < aisle:
						self.SetTargetVelocities(0, 0.3) # Rotate left, then drive forwards
						time.sleep(2)
						self.SetTargetVelocities(0.03, 0)
						time.sleep(2)
					elif current_aisle > aisle:
						self.SetTargetVelocities(0, -0.3) # Rotate right, then drive forwards
						time.sleep(2)
						self.SetTargetVelocities(0.03, 0)
						time.sleep(2)
				
				if proximity < 0.3:
					while proximity < 0.3:
						print("In obstacle avoidance state...")
						self.SetTargetVelocities(0, -0.2) # Drive backwards
						time.sleep(0.1)  # Assume a 100 ms sleep duration
						proximity = self.readProximity()  # Update proximity
						print(f"Current proximity: {proximity}")
					state = orient 


				proximity = self.readProximity() # Get the ranges from the ultrasonic sensor
				if proximity >= 0.3:
					print("Safe distance, looking for row marker")
					self.SetTargetVelocities(0.02, 0) # Drive forwards
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
			angular_velocity = -0.15 # Rotate right
		else: # odd number shelf
			angular_velocity = 0.15 # Rotate left

		while True:
			initial_bearing = rowdetect.get_detected_row_marker_bearing()

			if initial_bearing is not None:
				break

			self.SetTargetVelocities(0, angular_velocity)
			time.sleep(0.1)

		self.SetTargetVelocities(0, 0) # Stop
		self.exitnav()

	def aligning(self):
		# Stage 1: Locate the Packing Bay
		while True:
			packing_bay_rb = self.GetDetectedPackingBay()
			if packing_bay_rb:
				self.SetTargetVelocities(0, 0)  # Stop rotating
				break
			else:
				self.SetTargetVelocities(0, 0.3)  # Rotate until detected
			time.sleep(0.1)

		# Stage 2: Align with Rightmost Edge
		while True:
			packing_bay_rb = self.GetDetectedPackingBay()
			if not packing_bay_rb:
				self.SetTargetVelocities(0, 0)  # Stop rotating once out of view
				break
			else:
				self.SetTargetVelocities(0, -0.15)  # Rotate in the opposite direction
			time.sleep(0.1)
					

	def navigate_to_drop_off(self):
		# Stage 1: Rotate until the packing bay is centered
		while True:
			packing_bay_rb = self.GetDetectedPackingBay()
			if not packing_bay_rb:
				self.SetTargetVelocities(0, 0.3)  # Rotate if the packing bay is not in view
				continue
			_range, _bearing = packing_bay_rb
			angular_velocity = self.calculate_angular_velocity(_bearing)
			
			# If bearing is small enough (packing bay is centered), break to move to Stage 2
			if abs(_bearing) < 0.05:  # Assume 0.05 rad is sufficiently centered
				self.SetTargetVelocities(0, 0)  # Stop rotating
				break

			self.SetTargetVelocities(0, angular_velocity)
			time.sleep(0.1)

		# Stage 2: Drive straight towards the packing bay
		while True:
			packing_bay_rb = self.GetDetectedPackingBay()
			proximity = self.readProximity()
			if not packing_bay_rb:
				print("Lost sight of the packing bay while driving towards it!")
				break  # Or implement behavior to re-orient towards the packing bay

			_range, _bearing = packing_bay_rb
			
			# If we're close enough, stop
			if proximity < 0.2:  # Assume 0.5 m is a safe stopping distance
				self.SetTargetVelocities(0, 0)
				print("Reached the packing bay")
				break

			# Move towards the packing bay
			self.SetTargetVelocities(0.05, 0)  # Drive straight forward
			time.sleep(0.1)
		
		# Stage 3: Go to the drop off
		while True:
			proximity = self.readProximity()
			if proximity < 21:
				print("Reached the drop off")
				self.SetTargetVelocities(0, 0) # Stop

	def initAisle(self):
		self.currentAisle = -1

	def updatecurrentAisle(self):
		self.SetTargetVelocities(0, -0.2) # rotate 360 degrees until marker is seen
		
		while True:
			# Get blob info from the Vision class
			blob_info = rowdetect.vision.find_information()

			if blob_info is not None:
				blob_count = blob_info["blob_count"]

				# Check blob_count to determine current aisle
				if blob_count in [1, 2, 3]:  # Assuming aisles are represented by 1, 2, or 3 blobs respectively
					self.SetTargetVelocities(0, 0)
					self.currentAisle = blob_count - 1  # 0-indexed aisle number
					print(f"I am in aisle {self.currentAisle}")
					return  # Exit the function

			time.sleep(0.1)


	