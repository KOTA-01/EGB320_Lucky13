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
        GPIO.setup(6, GPIO.IN)  # Define GPIO pin 22 as an input (hori limit)

        # Set PWM frequencies as 50Hz
        self.pVert = GPIO.PWM(23, 50)
        self.pHori = GPIO.PWM(24, 50)
        
        #self.vertFwrd = 6.5
        #self.vertStop = 7
        #self.vertRvrs = 7.5
        self.vertFwrd = 6
        self.vertStop = 7.1
        self.vertRvrs = 7.75
        
        self.horzFwrd = 6.75
        self.horzStop = 7.1
        self.horzRvrs = 7.5

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
                self.pVert.ChangeDutyCycle(self.vertFwrd) 
                if GPIO.input(27):
                    print("s")
                    self.pVert.ChangeDutyCycle(self.vertStop) # Hold stop position
                    return True 
        elif level == "low" and not GPIO.input(17):
            self.pVert.start(5)
            while not GPIO.input(17):
                self.pVert.ChangeDutyCycle(self.vertRvrs)
                if GPIO.input(17):
                    self.pVert.ChangeDutyCycle(self.vertStop) # Hold stop position
                    return True
        else:
            self.pVert.ChangeDutyCycle(self.vertStop) # Hold stop position
            return True
        
    def Servo_Horz(self, spread):
        if spread == "wide" and not GPIO.input(22):
            self.pHori.start(5)
            while not GPIO.input(22):
                self.pHori.ChangeDutyCycle(self.horzFwrd) 
                if GPIO.input(22):
                    print("a")
                    self.pHori.ChangeDutyCycle(self.horzStop) # Hold stop position
                    return True 
        #elif spread == "narrow" and not GPIO.input(17):
        #    self.pHori.start(5)
        #    while not GPIO.input(17):
        #        self.pHori.ChangeDutyCycle(self.horzRvrs)
        #        if GPIO.input(17):
        #            self.pHori.ChangeDutyCycle(self.horzStop) # Hold stop position
        #            return True
        elif spread == "close" and not GPIO.input(22):
            self.pHori.start(5)
            while not GPIO.input(22):
                self.pHori.ChangeDutyCycle(self.horzRvrs)
                if GPIO.input(22):
                    self.pHori.ChangeDutyCycle(self.horzStop) # Hold stop position
                    return True
        else:
            self.pHori.ChangeDutyCycle(self.horzStop) # Hold stop position
            return True
        
    def Clean(self):
        GPIO.cleanup()
        print("Goodbye!")
        return True
    
    def Debug(self):
         while not GPIO.input(6):
                if GPIO.input(6):
                    print('a')
                    return True   

if __name__ == '__main__':
    GPIO.cleanup()
    #claw = Collection()
    claw.Upack()
    #claw.Deliver()
    #claw.Debug()
    

