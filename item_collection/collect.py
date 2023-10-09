# Primo's item collection script

# Libs
from gpiozero import AngularServo
from time import *
import RPi.GPIO as GPIO

class Collection:
    def __init__(self):
        # Pin SetUp
        GPIO.setup(23, GPIO.OUT)
        GPIO.setup(24, GPIO.OUT)

        # Set PWM frequencies as 50Hz
        self.pVert = GPIO.PWM(23, 50)
        self.pHori = GPIO.PWM(24, 50)

    def Upack(self):
        self.Servo_Vert(self,"high")
        #self.Servo_Horz(self,"wide")
        time.sleep(0.5)
        return True
        
    def Drift_mode(self):
        self.Servo_Vert(self,"high")
        time.sleep(0.5)
        return True

    def Deliver(self):
        self.Servo_Vert("low")
        time.sleep(0.5)
        self.Servo_Horz("wide")
        return True
    
    def Servo_Vert(self, level):
        if level == "high" & (not GPIO.input(27)):
            self.pVert.start(5)
            while not GPIO.input(27):
                self.pVert.ChangeDutyCycle(6.5) 
                if GPIO.input(27): 
                    self.pVert.ChangeDutyCycle(7) # Hold stop position
                    return True 
        elif level == "low" & (not GPIO.input (17)):
            self.pVert.start(5)
            while not GPIO.input(17):
                self.pVert.ChangeDutyCycle(7.5)
                if GPIO.input(27):
                    self.pVert.ChangeDutyCycle(7) # Hold stop position
                    return True
        else:
            self.pVert.ChangeDutyCycle(7) # Hold stop position
            return True
        
    def Servo_Horz(self, spread):
        if spread == "wide" & (not GPIO.input(27)):
            self.pHori.start(5)
            while not GPIO.input(27):
                self.pVert.ChangeDutyCycle(6.5) 
                if GPIO.input(27): 
                    self.pVert.ChangeDutyCycle(7) # Hold stop position
                    return True 
        elif spread == "narrow" & (not GPIO.input (17)):
            self.pVert.start(5)
            while not GPIO.input(17):
                self.pVert.ChangeDutyCycle(7.5)
                if GPIO.input(27):
                    self.pVert.ChangeDutyCycle(7) # Hold stop position
                    return True
        elif spread == "close" & (not GPIO.input (17)):
            self.pVert.start(5)
            while not GPIO.input(17):
                self.pVert.ChangeDutyCycle(7.5)
                if GPIO.input(27):
                    self.pVert.ChangeDutyCycle(7) # Hold stop position
                    return True
        else:
            self.pVert.ChangeDutyCycle(7) # Hold stop position
            return True
        
    def Clean(self):
        GPIO.cleanup()
        print("Goodbye!")
        return True

if __name__ == '__main__':
    claw = Collection()
    claw.Upack()