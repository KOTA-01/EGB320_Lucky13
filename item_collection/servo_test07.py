# WORKS 20231010: Turn vert 9G servo upwards in one direction until limit switch, then stop.

import RPi.GPIO as GPIO
import time
GPIO.setmode(GPIO.BCM)

# Set vertical servo as 23, horizontal servo as 24.
GPIO.setup(23, GPIO.OUT)
GPIO.setup(24, GPIO.OUT)

# Set PWM frequencies as 50Hz
pVert = GPIO.PWM(23, 50)
pHori = GPIO.PWM(24, 50)

# Set upper and lower limit switches
GPIO.setup(17,GPIO.IN) # lower
GPIO.setup(27,GPIO.IN) # upper

for i in range(1):
      #Ask user for shelf and turn servo to it
      shelf = float(input('Enter shelf (1, 2 or 3): '))
      if (shelf == 1):
        pVert.start(5)
        while not GPIO.input(17):
          pVert.ChangeDutyCycle(7.5) # lower
          if GPIO.input(17):
            break
      elif (shelf == 3):
        pVert.start(5)
        while not GPIO.input(27):
          pVert.ChangeDutyCycle(6.5) # raise
          if GPIO.input(27):
            break
      #time.sleep(0.5)
      pVert.ChangeDutyCycle(7) # Hold stop position


GPIO.cleanup()
print("Goodbye!")
