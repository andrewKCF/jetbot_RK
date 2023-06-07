#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64 #contro_effort
'''
from Raspi_MotorHAT import Raspi_MotorHAT, Raspi_DCMotor
mh = Raspi_MotorHAT(addr=0x60)
Motor_left=mh.getMotor(1)
Motor_right=mh.getMotor(2)
'''

import time
from Adafruit_MotorHAT import Adafruit_MotorHAT
from std_msgs.msg import String
motor_driver = Adafruit_MotorHAT(i2c_bus=1)
motor_left_ID = 1
motor_right_ID = 2
Motor_left = motor_driver.getMotor(motor_left_ID)
Motor_right = motor_driver.getMotor(motor_right_ID)


def callback(data):
    
    pwm=int(abs(data.data))
    print("contro_effort %", data.data)
    print("contro_effort %", pwm)
    if (data.data)==0:
        Motor_left.setSpeed(0)
        Motor_right.setSpeed(0)

    if (data.data)>0:
        print("left")
        Motor_left.setSpeed(pwm)
        Motor_right.setSpeed(pwm)
        Motor_left.run(Adafruit_MotorHAT.FORWARD)
        Motor_right.run(Adafruit_MotorHAT.BACKWARD)
    if (data.data)<0:
        print("right")
        Motor_left.setSpeed(pwm)
        Motor_right.setSpeed(pwm)
        Motor_left.run(Adafruit_MotorHAT.BACKWARD)
        Motor_right.run(Adafruit_MotorHAT.FORWARD)    

    
def listener():
    rospy.init_node('motor_contorl', anonymous=True)
    rospy.Subscriber("control_effort", Float64, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
