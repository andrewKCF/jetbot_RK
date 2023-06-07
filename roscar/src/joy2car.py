#!/usr/bin/env python
import rospy
import math
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from datetime import datetime
'''
from Adafruit_MotorHAT import Adafruit_MotorHAT
motor_driver = Adafruit_MotorHAT(i2c_bus=1)
motor_left_ID = 1
motor_right_ID = 2
Motor_left = motor_driver.getMotor(motor_left_ID)
Motor_right = motor_driver.getMotor(motor_right_ID)
from Adafruit_MotorHAT import Adafruit_MotorHAT, Raspi_DCMotor
# create a default object, no changes to I2C address or frequency
mh = Adafruit_MotorHAT(addr=0x60)

Motor_left=mh.getMotor(1)
Motor_right=mh.getMotor(2)
'''

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

from DFRobot_RaspberryPi_DC_Motor import DFRobot_DC_Motor_IIC as Board
board = Board(1, 0x10)
board.set_encoder_enable(board.ALL)
board.set_encoder_reduction_ratio(board.ALL, 43)
board.set_moter_pwm_frequency(1000)

def callback(data):
        print('stick left LR 0:%s'%data.axes[0])
        print('stick left UD 1:%s'%data.axes[1])
        print('stick right LR 2:%s'%data.axes[2])                
        print('stick right UD 3:%s'%data.axes[3])                
        print('R2:%s'%data.axes[4])
        print('L2:%s'%data.axes[5])
        print('cross key LR:{:.50f}'.format(data.axes[6]))
        print('cross key UD:%s'%data.axes[7])
        

        print('A:%s'%data.buttons[0])
        print('B:%s'%data.buttons[1])
        #print('2:%s'%data.buttons[2])
        print('X:%s'%data.buttons[3])
        print('Y:%s'%data.buttons[4])
        #print('5:%s'%data.buttons[5])
        print('L1:%s'%data.buttons[6])
        print('R1:%s'%data.buttons[7])
        print('L2:%s'%data.buttons[8])
        print('R2:%s'%data.buttons[9])
        print('SELECT:%s'%data.buttons[10])
        print('START:%s'%data.buttons[11])
        #print('12:%s'%data.buttons[12])
        print('stick left bu:%s'%data.buttons[13])
        print('stick right bu:%s'%data.buttons[14])
        now=datetime.now()
        current_time=now.strftime("%H:%M:%S")
        print("current time:",current_time)
        

        if (data.axes[6]!=0 or data.axes[7]!=0):
                if (data.axes[7]>0):
                        print("forward")
                        board.motor_movement([board.M1], board.CW, 100)
                        board.motor_movement([board.M2], board.CW, 100)   # DC motor 2 movement, orientation count-clockwise
                elif(data.axes[7]<0):
                        print("backward")
                        board.motor_movement([board.M1], board.CCW, 100)
                        board.motor_movement([board.M2], board.CCW, 100)   # DC motor 2 movement, orientation count-clockwise
                if (data.axes[6]>0):
                        print("left")
                        board.motor_movement([board.M1], board.CCW, 100)
                        board.motor_movement([board.M2], board.CW, 100)   # DC motor 2 movement, orientation count-clockwise
                elif(data.axes[6]<0):
                        print("right")
                        board.motor_movement([board.M1], board.CW, 100)
                        board.motor_movement([board.M2], board.CCW, 100)   # DC motor 2 movement, orientation count-clockwise
        else:
                board.motor_movement([board.M1], board.CW,0)
                board.motor_movement([board.M2], board.CW,0)
                print("End")
# Intializes everything
def start():
	# subscribed to joystick inputs on topic "joy"
	rospy.Subscriber("joy", Joy, callback)
	# starts the node
	rospy.init_node('joy2car')
	rospy.spin()

if __name__ == '__main__':
        print("joy2car START")
	start()
