import time
import cv2
import numpy as np
import math
from itemIndex import item_to_index
from Read_order_class import OrderReader
from bayDistanceIndex import distance_from_wall
from enum import IntEnum
from shelf_aisle_index import WarehouseLayout
from orient_avoidance import ObstacleDetectedException
from Motor_Task import stop
from Motor_Task import Motor
from Motor_Task import turn_indefinitely
from Motor_Task import steering
from ultrasonic import GroveUltrasonicRanger
from visionpivr import Vision
from navigation.collection04 import Collection
claw = Collection()
ultra = GroveUltrasonicRanger(26)
layout = WarehouseLayout()
order_reader = OrderReader()
vision = Vision()
vision.distance()   
vision.display()
vision.Test()
import sys
import os
sys.path.append("../")
import time
from DFRobot_RaspberryPi_DC_Motor import THIS_BOARD_TYPE, DFRobot_DC_Motor_IIC as Board

if THIS_BOARD_TYPE:
  board = Board(1, 0x10)    # RaspberryPi select bus 1, set address to 0x10
else:
  board = Board(7, 0x10)    # RockPi select bus 7, set address to 0x10

def board_detect():
  l = board.detecte()
  print("Board list conform:")
  print(l)
''' print last operate status, users can use this variable to determine the result of a function call. '''
def print_board_status():
  if board.last_operate_status == board.STA_OK:
    print("board status: everything ok")
  elif board.last_operate_status == board.STA_ERR:
    print("board status: unexpected error")
  elif board.last_operate_status == board.STA_ERR_DEVICE_NOT_DETECTED:
    print("board status: device not detected")
  elif board.last_operate_status == board.STA_ERR_PARAMETER:
    print("board status: parameter error, last operate no effective")
  elif board.last_operate_status == board.STA_ERR_SOFT_VERSION:
    print("board status: unsupport board framware version")

if __name__ == "__main__":
  board_detect()    # If you forget address you had set, use this to detected them, must have class instance
  # Set board controler address, use it carefully, reboot module to make it effective
  '''
  board.set_addr(0x10)
  if board.last_operate_status != board.STA_OK:
    print("set board address faild")
  else:
    print("set board address success")
  '''

  while board.begin() != board.STA_OK:    # Board begin and check board status
    print_board_status()
    print("board begin faild")
    time.sleep(2)
  print("board begin success")

  board.set_encoder_enable(board.ALL)                 # Set selected DC motor encoder enable
  # board.set_encoder_disable(board.ALL)              # Set selected DC motor encoder disable
  board.set_encoder_reduction_ratio(board.ALL, 43)    # Set selected DC motor encoder reduction ratio, test motor reduction ratio is 43.8
  board.set_moter_pwm_frequency(1000)   # Set DC motor pwm frequency to 1000HZ

current_order = order_reader.ReadOrder("Order_1.csv")

class robot(object): 
    def __init__(self):
        print("Robot")
        #current_order = (1, 2, 3, 1, ball)

        if (operation(current_order) == True):
            current_order = order_reader.next_order()
            if (operation(current_order) == True):
                current_order = order_reader.next_order()
                if (operation(current_order) == True):
                    print("done")

         
    # Function Navigates the Robot down the Asile
    def Operations(self, Aisle, Height, Section): 
        time.sleep(1) 
        Scanning = 0        
        lib = []

        while (True): 
            dot_success, Dots_detected, dot_bearing, dot_distance = vision.Aisle()
            if (Dots_detected == Aisle):
                Scanning = 0
                if (dot_bearing > 3):
                    turn_indefinitely("Left") # rotate 360 degrees
                elif (dot_bearing < -3):
                    turn_indefinitely("Right") # rotate 360 degrees                           
                else:
                    Motor("Forward_60")
                    if (Section == 3):
                        desired_distance = 81 # section 3 clostest to wall
                    elif (Section == 2):
                        desired_distance = 42 # section 2
                    elif (Section == 1):
                        desired_distance = 23 # section 1
                    elif (Section == 0):
                        desired_distance = 18 # section 0
                    else:
                        print("error")
                    
                    if (dot_distance > desired_distance):                               
                        stop()
                        
                        if (Aisle % 2) == 0:
                            side = 0
                            turn_indefinitely("Left") 
                        else:
                            side = 1
                            turn_indefinitely("Right")
                        time.sleep(4)
                        stop()

                        while (True):
                            Object_Found, Object_bearing = vision.Object()
                            if (Object_Found == True):
                                if (Object_bearing > 1):
                                    turn_indefinitely("Left") # rotate 360 degrees
                                elif (Object_bearing < -1):
                                    turn_indefinitely("Right") # rotate 360 degrees                           
                                else:
                                    stop()
                                    time.sleep(2)
                                    Motor("Backward_60")
                                    time.sleep(0.5)
                                    stop()
                                    #Arm rasie to height#   
                                    if (Height == 1):
                                        claw.Servo_Vert("low")
                                    elif (Height == 2):
                                        claw.Servo_Vert("mid")
                                    elif (Height == 2):
                                        claw.Servo_Vert("high")
                                    time.sleep(2)
                                    Motor("Forward_60")
                                    time.sleep(1)
                                    stop()
                                    claw.Servo_Horz("close_claw")
                                    time.sleep(2)
                                    Motor("Backward_60")
                                    time.sleep(1)
                                    stop()
                                    claw.Servo_Vert("low")
                                    time.sleep(1)
                                    
                                    while True:
                                        dot_success, Dots_detected, dot_bearing, dot_distance = vision.Aisle()
                                        if (Dots_detected > 0):
                                            if (dot_bearing > 3):
                                                turn_indefinitely("Left") # rotate 360 degrees
                                            elif (dot_bearing < -3):
                                                turn_indefinitely("Right") # rotate 360 degrees                           
                                            else:
                                                Motor("Forward_60")
                                                if (dot_distance > 60):                            
                                                    stop()    
                                                    time.sleep(1)                                                                                    
                                                    turn_indefinitely("Left")
                                                    time.sleep(5)
                                                    Found_true = 0
                                                    while (True):
                                                        shelf_success, shelf_count, shelf_bearing = vision.Shelves()
                                                        if (shelf_count == 2):
                                                            Found_true = 1
                                                            if (shelf_bearing > 2):
                                                                turn_indefinitely("Left") # rotate 360 degrees
                                                            elif (shelf_bearing < -2):
                                                                turn_indefinitely("Right") # rotate 360 degrees                            
                                                            else:
                                                                Motor("Forward_60")                                                                    
                                                        elif(Found_true == 1):
                                                            Motor("Forward_60") 
                                                            time.sleep(4)
                                                            stop()
                                                            yellow_found = 0
                                                            while True:
                                                                yellow_success, yellow_bearing = vision.PackZone()
                                                                if (yellow_success == True):
                                                                    yellow_found = 1
                                                                    if (yellow_bearing > 2):
                                                                        turn_indefinitely("Left") # rotate 360 degrees
                                                                    elif (yellow_bearing < -2):
                                                                        turn_indefinitely("Right") # rotate 360 degrees                            
                                                                    else:
                                                                        Motor("Forward_60")

                                                                elif(yellow_success == False):
                                                                    print(yellow_found)
                                                                    if(yellow_found == 1):
                                                                        Motor("Forward_60") 
                                                                        time.sleep(3.5)
                                                                        stop()
                                                                        time.sleep(2) 
                                                                        claw.Deliver()
                                                                        Motor("Backward_60")
                                                                        time.sleep(4)
                                                                        stop()
                                                                        return True
                                                    
                                                                    else:                                                                                                                                                        
                                                                        turn_indefinitely("Right")                                                                            
                                                        else:
                                                            turn_indefinitely("Left")
                                    
                                        else:
                                            turn_indefinitely("Right")    
                            else:
                                if (side == 0):
                                    turn_indefinitely("Right")
                                elif (side == 1):
                                    turn_indefinitely("Left")
        
            elif (Dots_detected == 0):                              
                turn_indefinitely("Left") # rotate 360 degrees
                Scanning += 1 
                print(Scanning)
                if (Scanning >= 200):
                    stop()
                    time.sleep(1) 
                    while True:
                        yellow_success, yellow_bearing = vision.PackZone()
                        turn_indefinitely("Left") # rotate 360 degrees
                        if(yellow_success == True):
                            stop()
                            time.sleep(2)
                            Motor("Forward_60")
                            time.sleep(3)
                            Scanning = 0
                            break 
            
            elif (Dots_detected > Aisle):
                if (Dots_detected != 0):
                    lib.append(Dots_detected)
                if (len(lib) == 5):
                    Scanning = 0
                    turn_indefinitely("Left") 
                    time.sleep(4)
                    stop()
                    Motor("Forward_60")
                    time.sleep(5)
                    stop()
                    lib = []
                    
            elif (Dots_detected < Aisle):
                if (Dots_detected != 0):
                    lib.append(Dots_detected)
                if (len(lib) == 5):
                    Scanning = 0
                    turn_indefinitely("Right") 
                    time.sleep(4)
                    stop()
                    Motor("Forward_60")
                    time.sleep(5)
                    stop()
                    lib = []
                                   
            
