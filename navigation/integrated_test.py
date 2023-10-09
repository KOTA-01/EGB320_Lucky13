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
from ultrasonic import UltrasonicSensor

TRIG = 26
ECHO = 19

sensor = UltrasonicSensor(TRIG, ECHO)

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
						proximity = sensor.get_distance() # Get the ranges from the ultrasonic sensor

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
						proximity = sensor.get_distance() # Get the ranges from the ultrasonic sensor

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
							stop() # Find the motions needed stop
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
						proximity = sensor.get_distance() # Get the ranges from the ultrasonic sensor
						closest_shelf = self.getClosestShelf() # Return if shelf is present in view
						current_aisle = self.updatecurrentAisle()
						rowMaker = rowdetect.GetDetectedRowMarker()
						if closest_shelf:
							if proximity <= 0.5:
								print("In reposition state: avoiding obstacle...")
								if current_aisle < self.previous_aisle:
									proximity = sensor.get_distance() # Get the ranges from the ultrasonic sensor
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
									proximity = sensor.get_distance() # Get the ranges from the ultrasonic sensor
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
										proximity = sensor.get_distance()
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
					range_ultrasonic = sensor.get_distance()
					range_camera = rowdetect.get_detected_row_marker_range()
					print("The range is: %0.4f, %0.4f" %(range_ultrasonic), (range_camera))


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

	def exitnav(self):
		target_distance = 1.1  # Distance you want to be from the row marker
		linear_tolerance = 0.02
		angular_tolerance = 0.05
		time.sleep(2)

		while True:
			initial_bearing = self.GetDetectedRowMarker_bearing() # row marker bearing
			
			if not initial_bearing or abs(initial_bearing) < angular_tolerance:
				break
			
			angular_velocity = -0.05 if initial_bearing < 0 else 0.05
			steering(0, angular_velocity)
			time.sleep(0.1)

		# 3. Drive and Adjust based on Row Marker's bearing and distance
		while True:
			row_marker_bearing = self.GetDetectedRowMarker_bearing()
			current_range = sensor.get_distance() # connect to the ultrasonic

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
			turn_indefinitly()
			while True:
				retCode, objectsDetected, _, _, _ = coppelia.simxCallScriptFunction(
					self.clientID, 'Robot', coppelia.sim_scripttype_childscript, 
					'getObjectsInView', [], [], [], bytearray(), coppelia.simx_opmode_blocking
				)

				if retCode == coppelia.simx_return_ok:
					for i, position in enumerate(self.rowMarkerPositions):
						if position is not None and objectsDetected[warehouseObjects.row_marker_1 + i]:
							stop()
							self.currentAisle = i
							print("I am in aisle %0.4f" %(i))
							return i #Exit the function
				time.sleep(0.1)