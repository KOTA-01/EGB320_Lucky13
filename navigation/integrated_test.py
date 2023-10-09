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
from mobility.motor_control import turn_indefinitly
from mobility.motor_control import steering

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
							stop() # Stop
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
									Motor("RotateL_90") # 90 degrees Left
									steering(0.08, 0)
									time.sleep(2) # Find the motions needed forwards slow
									turn_indefinitly("Right")

									if rowMaker:
										stop() 
										state = initialise
										break
												

								elif current_aisle > self.previous_aisle:
									proximity = self.readProximity() # Get the ranges from the ultrasonic sensor
									current_aisle = self.updatecurrentAisle()
									rowMaker = rowdetect.GetDetectedRowMarker()
									Motor("RotateR_90") # 90 degrees Right
									steering(0.08, 0)
									time.sleep(2) # Find the motions needed forwards slow
									turn_indefinitly("Left")
									if rowMaker:
										stop  # Find the motions needed stop
										state = initialise
										break
								
								elif current_aisle == self.previous_aisle:
									while proximity < 0.5:
										proximity = self.readProximity()
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