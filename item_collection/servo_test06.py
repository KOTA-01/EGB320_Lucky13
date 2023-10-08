# WORKS (20231008): Turn each 9G servo in one direction
# then stop then turn it in the other direction then stop.

import RPi.GPIO as GPIO
import time
GPIO.setmode(GPIO.BCM)

# Set vertical servo as 23, horizontal servo as 24.
GPIO.setup(23, GPIO.OUT)
GPIO.setup(24, GPIO.OUT)

# Set PWM frequencies as 50Hz
pVert = GPIO.PWM(23, 50)
pHori = GPIO.PWM(24, 50)

# Turn vert servo in one direction, stop, turn other direction, stop.
pVert.start(5)
pVert.ChangeDutyCycle(8)
time.sleep(2)
pVert.ChangeDutyCycle(7) # Hold stop position
time.sleep(2)
pVert.ChangeDutyCycle(6)
time.sleep(2)
pVert.ChangeDutyCycle(7) # Hold stop position

# Wait 3 seconds
time.sleep(3)

# Turn hori servo in one direction, stop, turn other direction, stop.
pHori.start(5)
pHori.ChangeDutyCycle(8)
time.sleep(2)
pHori.ChangeDutyCycle(7) # Hold stop position
time.sleep(2)
pHori.ChangeDutyCycle(6)
time.sleep(2)
pHori.ChangeDutyCycle(7) # Hold stop position

GPIO.cleanup()
