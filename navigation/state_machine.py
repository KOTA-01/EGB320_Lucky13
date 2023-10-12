
import time
from itemIndex import item_to_index
from Read_order_class import OrderReader
from aisle_nav import robot

if __name__ == '__main__':

	# Wrap everything in a try except case that catches KeyboardInterrupts. 
	# In the exception catch code attempt to Stop the CoppeliaSim so don't have to Stop it manually when pressing CTRL+C
	try:

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
				state = drive
			
			elif state == drive:
				bot.nav_to_aisle()
				state = aisleNav

			elif state == aisleNav:
				bot.run()
				state = exit_aisle

			elif state == exit_aisle:
				print("Exiting aisle")
				bot.exiting()
				state = drop_off

			elif state == drop_off:
				print("Searching for Packing bay")
				bot.navigate_to_drop_off()
				state = done

			elif state == done:
				print("Completed :3")
				self.SetTargetVelocities(0, 0)
				break

	except KeyboardInterrupt as e:
		motor.SetTargetVelocities(0, 0)