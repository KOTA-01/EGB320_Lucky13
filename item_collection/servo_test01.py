# This is to test a 9g servo.

from gpiozero import AngularServo
from time import sleep

servo = AngularServo(18, min_pulse_width=0.0006, max_pulse_width=0.0023)

# Make the servo move between its minimum, maximum, and mid-point positions with a pause between each.
while (True):
    servo.angle = 90
    sleep(2)
    servo.angle = 0
    sleep(2)
    servo.angle = -90
    sleep(2)
