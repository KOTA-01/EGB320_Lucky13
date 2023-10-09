# -*- coding:utf-8 -*-
'''!
  @file DC_Motor_Demo.py
  @brief Connect board with raspberryPi.
  @n Make board power and motor connection correct.
  @n Run this demo.
  @n Motor 1 will move slow to fast, orientation clockwise, 
  @n motor 2 will move fast to slow, orientation count-clockwise, 
  @n then fast to stop. loop in few seconds.
  @n Motor speed will print on terminal
  @copyright  Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
  @license    The MIT License (MIT)
  @author     [tangjie](jie.tang@dfrobot.com)
  @version    V1.0.1
  @date       2022-04-19
  @url  https://github.com/DFRobot/DFRobot_RaspberryPi_Motor
'''
from __future__ import print_function
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
  
  
  
  
### ===== MOTOR MOVEMENTS ===== ###

"""
# Timer - (Only for forward and backwards)
stop_time = time(s) for stop time
# Forward
Forward_100: Move forward 100%
Forward_80: Move forward 80%
Forward_60: Move forward 60%
Forward_40: Move forward 40% Find this min value


# Backward
Backward_100: Move backwards 100%
Backward_80: Move backwards 80%
Backward_60: Move backwards 60%
Backward_40: Move backwards 40%




# Turn Left and Right
RotateL For Left and RotateR for Right
RotateL_30: Rotate on the spot 30 degrees left/right
RotateL_60: Rotate on the spot 60 degrees left/right
RotateL_90: Rotate on the spot 90 degrees left/right
RotateL_120: Rotate on the spot 120 degrees left/right
RotateL_150: Rotate on the spot 150 degrees left/right
RotateL_180: Rotate on the spot 180 degrees left/right
RotateL_210: Rotate on the spot 210 degrees left/right
RotateL_240: Rotate on the spot 240 degrees left/right
RotateL_270: Rotate on the spot 270 degrees left/right
RotateL_300: Rotate on the spot 300 degrees left/right
RotateL_330: Rotate on the spot 330 degrees left/right
RotateL_360: Rotate on the spot 360 degrees left/right


"""
import time
import threading


# Create a timer that executes the welcome() function after 5 seconds
def stop():
  board.motor_movement([board.M1], board.CW, 0)
  board.motor_movement([board.M2], board.CW, 0)

stop_time = 30
timer = threading.Timer(stop_time, stop)
timer.start()


def Motor(Task):

  # Forward
  if Task == "Forward_100":
     board.motor_movement([board.M1], board.CCW, 100)
     board.motor_movement([board.M2], board.CCW, 100)
     
  if Task == "Forward_80":
     board.motor_movement([board.M1], board.CCW, 80)
     board.motor_movement([board.M2], board.CCW, 80)
     
  if Task == "Forward_60":
     board.motor_movement([board.M1], board.CCW, 60)
     board.motor_movement([board.M2], board.CCW, 60)
     
  if Task == "Forward_40":
     board.motor_movement([board.M1], board.CCW, 40)
     board.motor_movement([board.M2], board.CCW, 40)
     
     
  # Backward
  if Task == "Backward_100":
     board.motor_movement([board.M1], board.CW, 100)
     board.motor_movement([board.M2], board.CW, 100)
     
  if Task == "Backward_80":
     board.motor_movement([board.M1], board.CW, 80)
     board.motor_movement([board.M2], board.CW, 80)
     
  if Task == "Backward_60":
     board.motor_movement([board.M1], board.CW, 60)
     board.motor_movement([board.M2], board.CW, 60)
     
  if Task == "Backward_40":
     board.motor_movement([board.M1], board.CW, 40)
     board.motor_movement([board.M2], board.CW, 40)
     
     
  # Rotate
  if Task == "RotateR_30":
     board.motor_movement([board.M1], board.CCW, 50)
     board.motor_movement([board.M2], board.CW, 50)
     time.sleep(0.82*1/3)
     board.motor_movement([board.M1], board.CCW, 0)
     board.motor_movement([board.M2], board.CW, 0)
     
  if Task == "RotateR_60":
     board.motor_movement([board.M1], board.CCW, 50)
     board.motor_movement([board.M2], board.CW, 50)
     time.sleep(0.82*2/3)
     board.motor_movement([board.M1], board.CCW, 0)
     board.motor_movement([board.M2], board.CW, 0)
     
  if Task == "RotateR_90":
     board.motor_movement([board.M1], board.CCW, 50)
     board.motor_movement([board.M2], board.CW, 50)
     time.sleep(0.82)
     board.motor_movement([board.M1], board.CCW, 0)
     board.motor_movement([board.M2], board.CW, 0)
     
  if Task == "RotateR_120":
     board.motor_movement([board.M1], board.CCW, 50)
     board.motor_movement([board.M2], board.CW, 50)
     time.sleep(0.82*4/3)
     board.motor_movement([board.M1], board.CCW, 0)
     board.motor_movement([board.M2], board.CW, 0)
     
  if Task == "RotateR_150":
     board.motor_movement([board.M1], board.CCW, 50)
     board.motor_movement([board.M2], board.CW, 50)
     time.sleep(0.82*5/3)
     board.motor_movement([board.M1], board.CCW, 0)
     board.motor_movement([board.M2], board.CW, 0)
     
  if Task == "RotateR_180":
     board.motor_movement([board.M1], board.CCW, 50)
     board.motor_movement([board.M2], board.CW, 50)
     time.sleep(0.82*2)
     board.motor_movement([board.M1], board.CCW, 0)
     board.motor_movement([board.M2], board.CW, 0)
     
  if Task == "RotateR_210":
     board.motor_movement([board.M1], board.CCW, 50)
     board.motor_movement([board.M2], board.CW, 50)
     time.sleep(0.82*7/3)
     board.motor_movement([board.M1], board.CCW, 0)
     board.motor_movement([board.M2], board.CW, 0)
     
  if Task == "RotateR_240":
     board.motor_movement([board.M1], board.CCW, 50)
     board.motor_movement([board.M2], board.CW, 50)
     time.sleep(0.82*8/3)
     board.motor_movement([board.M1], board.CCW, 0)
     board.motor_movement([board.M2], board.CW, 0)
     
  if Task == "RotateR_270":
     board.motor_movement([board.M1], board.CCW, 50)
     board.motor_movement([board.M2], board.CW, 50)
     time.sleep(0.82*3)
     board.motor_movement([board.M1], board.CCW, 0)
     board.motor_movement([board.M2], board.CW, 0)
     
  if Task == "RotateR_300":
     board.motor_movement([board.M1], board.CCW, 50)
     board.motor_movement([board.M2], board.CW, 50)
     time.sleep(0.82*10/3)
     board.motor_movement([board.M1], board.CCW, 0)
     board.motor_movement([board.M2], board.CW, 0)
     
  if Task == "RotateR_330":
     board.motor_movement([board.M1], board.CCW, 50)
     board.motor_movement([board.M2], board.CW, 50)
     time.sleep(0.82*11/3)
     board.motor_movement([board.M1], board.CCW, 0)
     board.motor_movement([board.M2], board.CW, 0)
     
  if Task == "RotateR_360":
     board.motor_movement([board.M1], board.CCW, 50)
     board.motor_movement([board.M2], board.CW, 50)
     time.sleep(3.5)
     board.motor_movement([board.M1], board.CCW, 0)
     board.motor_movement([board.M2], board.CW, 0)
     
     
     
     
     
  if Task == "RotateL_30":
     board.motor_movement([board.M1], board.CW, 50)
     board.motor_movement([board.M2], board.CCW, 50)
     time.sleep(0.82*1/3)
     board.motor_movement([board.M1], board.CW, 0)
     board.motor_movement([board.M2], board.CCW, 0)
     
  if Task == "RotateL_60":
     board.motor_movement([board.M1], board.CW, 50)
     board.motor_movement([board.M2], board.CCW, 50)
     time.sleep(0.82*2/3)
     board.motor_movement([board.M1], board.CW, 0)
     board.motor_movement([board.M2], board.CCW, 0)
     
  if Task == "RotateL_90":
     board.motor_movement([board.M1], board.CW, 50)
     board.motor_movement([board.M2], board.CCW, 50)
     time.sleep(0.82)
     board.motor_movement([board.M1], board.CW, 0)
     board.motor_movement([board.M2], board.CCW, 0)
     
  if Task == "RotateL_120":
     board.motor_movement([board.M1], board.CW, 50)
     board.motor_movement([board.M2], board.CCW, 50)
     time.sleep(0.82*4/3)
     board.motor_movement([board.M1], board.CW, 0)
     board.motor_movement([board.M2], board.CCW, 0)
     
  if Task == "RotateL_150":
     board.motor_movement([board.M1], board.CW, 50)
     board.motor_movement([board.M2], board.CCW, 50)
     time.sleep(0.82*5/3)
     board.motor_movement([board.M1], board.CW, 0)
     board.motor_movement([board.M2], board.CCW, 0)
     
  if Task == "RotateL_180":
     board.motor_movement([board.M1], board.CW, 50)
     board.motor_movement([board.M2], board.CCW, 50)
     time.sleep(0.82*2)
     board.motor_movement([board.M1], board.CW, 0)
     board.motor_movement([board.M2], board.CCW, 0)
     
  if Task == "RotateL_210":
     board.motor_movement([board.M1], board.CW, 50)
     board.motor_movement([board.M2], board.CCW, 50)
     time.sleep(0.82*7/3)
     board.motor_movement([board.M1], board.CW, 0)
     board.motor_movement([board.M2], board.CCW, 0)
     
  if Task == "RotateL_240":
     board.motor_movement([board.M1], board.CW, 50)
     board.motor_movement([board.M2], board.CCW, 50)
     time.sleep(0.82*8/3)
     board.motor_movement([board.M1], board.CW, 0)
     board.motor_movement([board.M2], board.CCW, 0)
     
  if Task == "RotateL_270":
     board.motor_movement([board.M1], board.CW, 50)
     board.motor_movement([board.M2], board.CCW, 50)
     time.sleep(0.82*3)
     board.motor_movement([board.M1], board.CW, 0)
     board.motor_movement([board.M2], board.CCW, 0)
     
  if Task == "RotateL_300":
     board.motor_movement([board.M1], board.CW, 50)
     board.motor_movement([board.M2], board.CCW, 50)
     time.sleep(0.82*10/3)
     board.motor_movement([board.M1], board.CW, 0)
     board.motor_movement([board.M2], board.CCW, 0)
     
  if Task == "RotateL_330":
     board.motor_movement([board.M1], board.CW, 50)
     board.motor_movement([board.M2], board.CCW, 50)
     time.sleep(0.82*11/3)
     board.motor_movement([board.M1], board.CW, 0)
     board.motor_movement([board.M2], board.CCW, 0)
     
  if Task == "RotateL_360":
     board.motor_movement([board.M1], board.CW, 50)
     board.motor_movement([board.M2], board.CCW, 50)
     time.sleep(3.5)
     board.motor_movement([board.M1], board.CW, 0)
     board.motor_movement([board.M2], board.CCW, 0)


def turn_L(t_time):
  board.motor_movement([board.M1], board.CW, 50)
  board.motor_movement([board.M2], board.CCW, 50)
  time.sleep(t_time)
  board.motor_movement([board.M1], board.CW, 0)
  board.motor_movement([board.M2], board.CCW, 0)
  
def turn_R(t_time):
  board.motor_movement([board.M1], board.CCW, 50)
  board.motor_movement([board.M2], board.CW, 50)
  time.sleep(t_time)
  board.motor_movement([board.M1], board.CW, 0)
  board.motor_movement([board.M2], board.CCW, 0)

def turn_indefinitely(task):
    if task == "Right":
        board.motor_movement([board.M1], board.CCW, 50)
        board.motor_movement([board.M2], board.CW, 50)

    if task == "Left":
        board.motor_movement([board.M1], board.CW, 50)
        board.motor_movement([board.M2], board.CCW, 50)




def steering(x_dot, theta_dot):
  wheelbase = 0.35
  wheelradius = 0.0175
    
  
  # wheel speed in rad/s
  LWS = (x_dot - 0.5*theta_dot*wheelbase)/(wheelradius)
  RWS = (x_dot + 0.5*theta_dot*wheelbase)/(wheelradius)
  
  print(LWS)
  print(RWS)
  
  # convert to rpm
  LWS_rpm = LWS * 9.5492968
  RWS_rpm = RWS * 9.5492968
  
  # scale to be less than 100
  if LWS_rpm > 100 or RWS_rpm > 100:
      upper = max(LWS_rpm, RWS_rpm)
      LWS_rpm = LWS_rpm/upper * 99
      RWS_rpm = RWS_rpm/upper * 99
  
  print(LWS_rpm)
  print(RWS_rpm)

  
  # send power to motors
  # Forward
  if LWS_rpm > 0 and RWS_rpm > 0:
    board.motor_movement([board.M1], board.CCW, abs(LWS_rpm))
    board.motor_movement([board.M2], board.CCW, abs(RWS_rpm))
    
  # Backward
  if LWS_rpm < 0 and RWS_rpm < 0:
    board.motor_movement([board.M1], board.CW, abs(LWS_rpm))
    board.motor_movement([board.M2], board.CW, abs(RWS_rpm))
    
  # Left
  if LWS_rpm < 0 and RWS_rpm > 0:
    board.motor_movement([board.M1], board.CW, abs(LWS_rpm))
    board.motor_movement([board.M2], board.CCW, abs(RWS_rpm))
    
  # Right
  if LWS_rpm > 0 and RWS_rpm < 0:
    board.motor_movement([board.M1], board.CCW, abs(LWS_rpm))
    board.motor_movement([board.M2], board.CW, abs(RWS_rpm))
    

    


    
    

  
  

