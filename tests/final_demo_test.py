from __future__ import print_function
import sys
import os
sys.path.append("../")

import time

from DFRobot_RaspberryPi_DC_Motor import THIS_BOARD_TYPE, DFRobot_DC_Motor_IIC as Board

if THIS_BOARD_TYPE:
  board = Board(1, 0x10)    # RaspberryPi select bus 1, set address to 0x10
else:
  board = Board(7, 0x10)    # RockPi select bus 7, set address to 0x10

def board_detect():
  l = board.detecte()
  print("Board list conform:")
  print(l)

''' print last operate status, users can use this variable to determine the result of a function call. '''
def print_board_status():
  if board.last_operate_status == board.STA_OK:
    print("board status: everything ok")
  elif board.last_operate_status == board.STA_ERR:
    print("board status: unexpected error")
  elif board.last_operate_status == board.STA_ERR_DEVICE_NOT_DETECTED:
    print("board status: device not detected")
  elif board.last_operate_status == board.STA_ERR_PARAMETER:
    print("board status: parameter error, last operate no effective")
  elif board.last_operate_status == board.STA_ERR_SOFT_VERSION:
    print("board status: unsupport board framware version")

if __name__ == "__main__":

  board_detect()    # If you forget address you had set, use this to detected them, must have class instance

  # Set board controler address, use it carefully, reboot module to make it effective
  '''
  board.set_addr(0x10)
  if board.last_operate_status != board.STA_OK:
    print("set board address faild")
  else:
    print("set board address success")
  '''

  while board.begin() != board.STA_OK:    # Board begin and check board status
    print_board_status()
    print("board begin faild")
    time.sleep(2)
  print("board begin success")

  board.set_encoder_enable(board.ALL)                 # Set selected DC motor encoder enable
  # board.set_encoder_disable(board.ALL)              # Set selected DC motor encoder disable
  board.set_encoder_reduction_ratio(board.ALL, 43)    # Set selected DC motor encoder reduction ratio, test motor reduction ratio is 43.8

  board.set_moter_pwm_frequency(1000)   # Set DC motor pwm frequency to 1000HZ

import time
from navigation.itemIndex import item_to_index
from navigation.Read_order_class import OrderReader
from navigation.aisle_nav import robot
from mobility.Motor_init import DFRobot_DC_Motor
from mobility.Motor_init import DFRobot_DC_Motor_IIC
from mobility.motor_control import stop

if __name__ == '__main__':

	# Wrap everything in a try except case that catches KeyboardInterrupts. 
	# In the exception catch code attempt to Stop the CoppeliaSim so don't have to Stop it manually when pressing CTRL+C
	try:

		DFRobot_DC_Motor()
		DFRobot_DC_Motor_IIC(DFRobot_DC_Motor)
		order_reader = OrderReader()
		bot = robot()
		# State machine initialization
		order = 2
		drive = 3
		aisleNav = 4
		exit_aisle = 5
		drop_off = 6
		done = 7

		state = order
		
		while True:
			if state == order:
				order_data = order_reader.ReadOrder("Order_1.csv")
				print(order_data)
				print("RED LED - I am searching for the item!")
				state = aisleNav
			
			elif state == drive:
				bot.nav_to_aisle()
				state = aisleNav

			elif state == aisleNav:
				bot.run()
				state = exit_aisle

			elif state == exit_aisle:
				print("Exiting aisle")
				bot.exiting()
				state = done

			elif state == drop_off:
				print("Searching for Packing bay")
				bot.navigate_to_packing_bay()
				state = done

			elif state == done:
				print("Completed :3")
				stop()
				break

	except KeyboardInterrupt as e:
		stop()

