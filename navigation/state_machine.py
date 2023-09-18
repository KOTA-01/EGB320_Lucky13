
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
		order = 1
		aisleNav = 2
		pick_item = 3

		state = order
		
		while True:

			if state == order:
				order_data = order_reader.ReadOrder("Order_1.csv")
				print(order_data)
				state = aisleNav

			elif state == aisleNav:
				bot.run()
				
				state = pick_item

			elif state == pick_item:
				(Where_the_motor_stuff_is).SetTargetVelocities(0, 0)
				"""Code for picking the item/calling to a function that does that"""

	except KeyboardInterrupt as e:
		(Where_the_motor_stuff_is).SetTargetVelocities(0, 0)