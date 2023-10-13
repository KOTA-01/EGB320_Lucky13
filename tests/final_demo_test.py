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

