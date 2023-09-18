import time
import cv2
import numpy as np
from itemIndex import item_to_index
from Read_order_class import OrderReader
order_reader = OrderReader()
from bayDistanceIndex import distance_from_wall
from enum import IntEnum

class robot(object):
	def bayNav(self, bay_number):
		target_distance = distance_from_wall(bay_number)
		linear_tolerance = 0.02
		angular_tolerance = 0.05
		lateral_tolerance = 0.05
		time.sleep(2)

		while True:
			self.UpdateObjectPositions()
			initial_bearing = self.GetDetectedRowMarker_bearing()
			
			if not initial_bearing or abs(initial_bearing) < angular_tolerance:
				break
			
			angular_velocity = -0.05 if initial_bearing < 0 else 0.05
			self.SetTargetVelocities(0, angular_velocity)
			time.sleep(0.1)

		# 3. Drive and Adjust based on Row Marker's bearing and distance
		while True:
			self.UpdateObjectPositions()
			row_marker_bearing = self.GetDetectedRowMarker_bearing()
			current_range = self.get_detected_row_marker_range()

			# Adjust orientation based on bearing
			if row_marker_bearing and abs(row_marker_bearing) > angular_tolerance:
				angular_velocity = -0.05 if row_marker_bearing < 0 else 0.05
			else:
				angular_velocity = 0

			# If we are within the acceptable range of the bay, stop
			if target_distance - linear_tolerance <= current_range <= target_distance + linear_tolerance:
				self.SetTargetVelocities(0, 0)
				break

			# If we pass the bay (i.e., too close to the end marker), reverse
			elif current_range < target_distance - linear_tolerance:
				linear_velocity = -0.05

			# Otherwise, drive forward towards the bay
			else:
				linear_velocity = 0.05
			
			self.SetTargetVelocities(linear_velocity, angular_velocity)
			time.sleep(0.1)

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
				row_marker_range = self.get_detected_row_marker_range()
				if row_marker_range is not None:
					range_init = self.get_detected_row_marker_range()
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

						shelf_number = int(current_order["shelf"])
						print("Go to shelf: %0.4f" %(shelf_number))

						if shelf_number % 2 == 0: # Even number shelf
							angular_velocity = 0.15
							self.SetTargetVelocities(0, angular_velocity)
							time.sleep(3)
							self.SetTargetVelocities(0,0)
						else: # odd number shelf
							angular_velocity = -0.15
							self.SetTargetVelocities(0, angular_velocity)
							time.sleep(3)
							self.SetTargetVelocities(0,0)
					
						state = at_bay
					else:
						print("Order not found tehehe")

				except Exception as e:
					print(f"Error reading order: {e}")
					state = marker

			elif state == at_bay:
				print("Picking up item ...")
				time.sleep(5) #Picking the item
				state = done
			
			if state == done:
				print("run complete :)")
				break

	def initAisle(self):
		self.currentAisle = -1

	def updatecurrentAisle(self):
			self.SetTargetVelocities(0, -0.2)
			while True:
				retCode, objectsDetected, _, _, _ = coppelia.simxCallScriptFunction(
					self.clientID, 'Robot', coppelia.sim_scripttype_childscript, 
					'getObjectsInView', [], [], [], bytearray(), coppelia.simx_opmode_blocking
				)

				if retCode == coppelia.simx_return_ok:
					for i, position in enumerate(self.rowMarkerPositions):
						if position is not None and objectsDetected[warehouseObjects.row_marker_1 + i]:
							self.SetTargetVelocities(0,0)
							self.currentAisle = i
							print("I am in aisle %0.4f" %(i))
							return #Exit the function
				time.sleep(0.1)

	