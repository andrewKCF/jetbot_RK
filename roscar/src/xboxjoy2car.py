#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from Raspi_MotorHAT import Raspi_MotorHAT, Raspi_DCMotor
from datetime import datetime
import math
# create a default object, no changes to I2C address or frequency
mh = Raspi_MotorHAT(addr=0x60)

Motor_left=mh.getMotor(1)
Motor_right=mh.getMotor(2)



def callback(data):
        print('stick left LR 0:%s'%data.axes[0])
        print('stick left UD 1:%s'%data.axes[1])
        print('stick right LR 2:%s'%data.axes[2])                
        print('stick right UD 3:%s'%data.axes[3])                    

        print('0 A:%s'%data.buttons[0])
        print('1 B:%s'%data.buttons[1])
        print('2 X:%s'%data.buttons[2])
        print('3 Y:%s'%data.buttons[3])
        print('4 LB:%s'%data.buttons[4])
        print('5 RB:%s'%data.buttons[5])
        print('6 LT:%s'%data.buttons[6])
        print('7 RT:%s'%data.buttons[7])
        print('8 BACK:%s'%data.buttons[8])
        print('9 START:%s'%data.buttons[9])
        print('10 X-RING:%s'%data.buttons[10])
        print('11 L-STICK:%s'%data.buttons[11])
        print('12 R-STICK:%s'%data.buttons[12])
        print('13 CLEFT:%s'%data.buttons[13])
        print('14 CRIGHT:%s'%data.buttons[14])
        print('15 CUP:%s'%data.buttons[15])
        print('16 CDOWN:%s'%data.buttons[16])

        now=datetime.now()
        current_time=now.strftime("%H:%M:%S")
        print("current time:",current_time)
        '''
        

        if (data.axes[6]!=0 or data.axes[7]!=0):
                if (data.axes[7]>0):
                        print("forward")
                        Motor_left.setSpeed(200)
                        Motor_right.setSpeed(200)
                        Motor_left.run(Raspi_MotorHAT.FORWARD)
                        Motor_right.run(Raspi_MotorHAT.FORWARD)
                elif(data.axes[7]<0):
                        print("backward")
                        Motor_left.setSpeed(200)
                        Motor_right.setSpeed(200)
                        Motor_left.run(Raspi_MotorHAT.BACKWARD)
                        Motor_right.run(Raspi_MotorHAT.BACKWARD)
                if (data.axes[6]>0):
                        print("left")
                        Motor_left.setSpeed(150)
                        Motor_right.setSpeed(150)
                        Motor_left.run(Raspi_MotorHAT.BACKWARD)
                        Motor_right.run(Raspi_MotorHAT.FORWARD)
                elif(data.axes[6]<0):
                        print("right")
                        Motor_left.setSpeed(150)
                        Motor_right.setSpeed(150)
                        Motor_left.run(Raspi_MotorHAT.FORWARD)
                        Motor_right.run(Raspi_MotorHAT.BACKWARD)
        else:
                Motor_left.setSpeed(0)
                Motor_right.setSpeed(0)
        '''

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
