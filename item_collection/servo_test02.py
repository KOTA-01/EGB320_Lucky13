from gpiozero import AngularServo
from time import sleep

# initialization
count = 1
n = 4 # exit limit
servo = AngularServo(18, min_pulse_width=0.0005, max_pulse_width=0.0020)
#servo =AngularServo(18, min_angle=0, max_angle=180, min_pulse_width=0.0005, max_pulse_width=0.0025)

# Make the servo move between its minimum, maximum, and mid-point positions with a pause between each.
servo.angle = 0
sleep(2)

servo.angle = -50
sleep(2)

servo.angle = 0 #100deg/2
sleep(2)

servo.angle = 50 #100deg/2
sleep(2)

servo.angle = 0 #100deg/2
sleep(2)


