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

class techniques(object):

	def __init__(self):
		self.angular_tolerance = 0.05
		pass

	def initialise_centre(self, angular_tolerance):
		print("initialising centre")
		initial_bearing = rowdetect.get_detected_row_marker_bearing()
		proximity = self.readProximity()

		if proximity < 0.5:
			print("repositioning to get a better entry")
			stop()
			return 'reposition'
			
		elif not initial_bearing or abs(initial_bearing) < 0.05:
			return 'drive'

		else:
			self.adjust_steering(initial_bearing)
			return 'initialise'
		
	def adjust_steering(self, initial_bearing):
		angular_velocity = -0.05 if initial_bearing < 0 else 0.05
		steering(0, angular_velocity)
		time.sleep(0.1)

	def adjust_and_drive(self, target_distance, linear_tolerance):
		row_marker_bearing = rowdetect.get_detected_row_marker_bearing()
		current_range = rowdetect.get_detected_row_marker_range()
		proximity = self.readProximity()

		if proximity < 0.5:
			print("repositioning for entry")
			return 'reposition'
		else:
			return self.drive(row_marker_bearing, current_range, target_distance, linear_tolerance)
		
	def drive(self, row_marker_bearing, current_range, target_distance, linear_tolerance):
		if row_marker_bearing and abs(row_marker_bearing) > 0.05:
			angular_velocity = -0.05 if row_marker_bearing < 0 else 0.05
		else:
			angular_velocity = 0

		if target_distance - linear_tolerance <= current_range <= target_distance + linear_tolerance:
			steering(0, 0)
			return 'complete'
		elif current_range < target_distance - linear_tolerance:
			linear_velocity = -0.08
		else:
			linear_velocity = 0.08

		steering(linear_velocity, angular_velocity)
		time.sleep(0.1)
		return 'drive'
	
	def navigate_and_avoid_obstacle(self, current_aisle, proximity, row_maker):
		if current_aisle < self.previous_aisle:
			return self.navigate("Left", "Right", proximity, row_maker)

		elif current_aisle > self.previous_aisle:
			return self.navigate("Right", "Left", proximity, row_maker)

		elif current_aisle == self.previous_aisle:
			return self.reverse_until_clear(proximity)

		else:
			return 'initialise'
		
	def navigate(self, rotate_direction, turn_direction, proximity, row_maker):
		Motor(f"Rotate{rotate_direction}_90")
		steering(0.08, 0)
		time.sleep(2)
		turn_indefinitely(turn_direction)

		if row_maker:
			stop()
			return 'initialise'
		else:
			return 'reposition'
	
	def reverse_until_clear(self, proximity):
		while proximity < 0.5:
			proximity = self.readProximity()
			Motor("Backward_40")
			if proximity >= 0.5:
				return 'initialise'
		return 'reposition'
	
	def reposition(self):
		print(f"previous aisle: {self.previous_aisle}")

		while True:
			proximity = self.readProximity()
			closest_shelf = self.getClosestShelf()
			current_aisle = self.update_current_aisle()
			row_maker = rowdetect.get_detected_row_marker()

			if closest_shelf and proximity <= 0.5:
				print("In reposition state: avoiding obstacle...")
				return self.navigate_and_avoid_obstacle(current_aisle, proximity, row_maker)





		