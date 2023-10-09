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
from functions import techniques

#change
class robot(object):

	def __init__(self):
		self.techniques = techniques()
	
	def bayNav(self, bay_number):
		target_distance = distance_from_wall(bay_number)
		linear_tolerance = 0.02

		initialise = 1
		reposition = 2
		adjust = 3
		drive = 4
		complete = 5
		
		state = initialise

		while True:
			if state == initialise:
				result = self.techniques.initialise_centre(angular_tolerance=0.05)
				if result == 'reposition':
					state = reposition
				elif result == 'drive':
					state = drive
			
			elif state == adjust:
				result = self.techniques.adjust_and_drive(
					target_distance=target_distance, 
					linear_tolerance=linear_tolerance
				)
				if result == 'reposition':
					state = reposition
				elif result == 'drive':
					state = drive

			elif state == drive:
				row_marker_bearing = rowdetect.get_detected_row_marker_bearing()
				current_range = rowdetect.get_detected_row_marker_range()
				result = self.techniques.drive(
					row_marker_bearing=row_marker_bearing, 
					current_range=current_range, 
					target_distance=target_distance, 
					linear_tolerance=linear_tolerance
				)
				if result == 'complete':
					state = complete

			elif state == reposition:
				result = self.techniques.reposition()
				if result == 'initialise':
					state = initialise
			
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
							return i #Exit the function
				time.sleep(0.1)
