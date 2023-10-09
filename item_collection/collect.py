# Primo's item collection script

# Libs
from gpiozero import AngularServo
from time import sleep
import RPi.GPIO as GPIO

class Collection:
    def __init__(self):
        # Pin SetUp
        GPIO.setmode(GPIO.BCM)  # Use BCM pin numbering
        GPIO.setup(23, GPIO.OUT)
        GPIO.setup(24, GPIO.OUT)
        GPIO.setup(27, GPIO.IN)  # Define GPIO pin 27 as an input (top limit)
        GPIO.setup(17, GPIO.IN)  # Define GPIO pin 17 as an input (bottom limit)
        GPIO.setup(22, GPIO.IN)  # Define GPIO pin 22 as an input (hori limit)

        # Set PWM frequencies as 50Hz
        self.pVert = GPIO.PWM(23, 50)
        self.pHori = GPIO.PWM(24, 50)

    def Upack(self):
        self.Servo_Vert("high")
        # self.Servo_Horz("wide")
        sleep(0.5)
        return True
        
    def Drift_mode(self):
        self.Servo_Vert("high")
        sleep(0.5)
        return True

    def Deliver(self):
        self.Servo_Vert("low")
        sleep(0.5)
        self.Servo_Horz("wide")
        return True
    
    def Servo_Vert(self, level):
        if level == "high" and not GPIO.input(27):
            self.pVert.start(5)
            while not GPIO.input(27):
                self.pVert.ChangeDutyCycle(6.5) 
                if GPIO.input(27): 
                    self.pVert.ChangeDutyCycle(7) # Hold stop position
                    return True 
        elif level == "low" and not GPIO.input(17):
            self.pVert.start(5)
            while not GPIO.input(17):
                self.pVert.ChangeDutyCycle(7.5)
                if GPIO.input(17):
                    self.pVert.ChangeDutyCycle(7) # Hold stop position
                    return True
        else:
            self.pVert.ChangeDutyCycle(7) # Hold stop position
            return True
        
    def Servo_Horz(self, spread):
        if spread == "wide" and not GPIO.input(27):
            self.pHori.start(5)
            while not GPIO.input(27):
                self.pHori.ChangeDutyCycle(6.5) 
                if GPIO.input(27): 
                    self.pHori.ChangeDutyCycle(7) # Hold stop position
                    return True 
        elif spread == "narrow" and not GPIO.input(17):
            self.pHori.start(5)
            while not GPIO.input(17):
                self.pHori.ChangeDutyCycle(7.5)
                if GPIO.input(17):
                    self.pHori.ChangeDutyCycle(7) # Hold stop position
                    return True
        elif spread == "close" and not GPIO.input(17):
            self.pHori.start(5)
            while not GPIO.input(17):
                self.pHori.ChangeDutyCycle(7.5)
                if GPIO.input(17):
                    self.pHori.ChangeDutyCycle(7) # Hold stop position
                    return True
        else:
            self.pHori.ChangeDutyCycle(7) # Hold stop position
            return True
        
    def Clean(self):
        GPIO.cleanup()
        print("Goodbye!")
        return True

if __name__ == '__main__':
    claw = Collection()
    claw.Upack()
