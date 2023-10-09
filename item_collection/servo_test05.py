import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)

# set the pin as input
GPIO.setup(17,GPIO.IN)
GPIO.setup(27,GPIO.IN)


inputBottom = GPIO.input(17)
inputTop = GPIO.input(27)

while True:
    if (GPIO.input(17)):
        print("Bottom pressed")
    elif (GPIO.input(27)):
        print("Top pressed")


