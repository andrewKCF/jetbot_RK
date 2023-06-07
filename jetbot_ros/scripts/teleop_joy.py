#!/usr/bin/env python
import rospy
import time

from Adafruit_MotorHAT import Adafruit_MotorHAT
from std_msgs.msg import String
from sensor_msgs.msg import Joy

# sets motor speed between [-1.0, 1.0]
def set_speed(motor_ID, value):
	max_pwm = 115.0
	speed = int(min(max(abs(value * max_pwm), 0), max_pwm))

	if motor_ID == 1:
		motor = motor_left
	elif motor_ID == 2:
		motor = motor_right
	else:
		rospy.logerror('set_speed(%d, %f) -> invalid motor_ID=%d', motor_ID, value, motor_ID)
		return
	
	motor.setSpeed(speed)

	if value > 0:
		motor.run(Adafruit_MotorHAT.FORWARD)
	else:
		motor.run(Adafruit_MotorHAT.BACKWARD)


# stops all motors
def all_stop():
	motor_left.setSpeed(0)
	motor_right.setSpeed(0)

	motor_left.run(Adafruit_MotorHAT.RELEASE)
	motor_right.run(Adafruit_MotorHAT.RELEASE)



# raw L/R motor commands (speed, speed)
def on_cmd_joy(msg):
	global mode
	scale=0.3
	if (mode=='manual'):
		#left +1, right -1 , up +1 , down -1
		rospy.loginfo("left-right:%f",msg.axes[0]*scale)
		rospy.loginfo("foward-back:%f",msg.axes[1]*scale)
		rospy.loginfo("button0:%d",msg.buttons[0])
		#left right
		#set_speed(motor_left_ID,  msg.axes[0])
		#set_speed(motor_right_ID, -msg.axes[0]) 
		#forward backward
		#set_speed(motor_left_ID,  -msg.axes[1])
		#set_speed(motor_right_ID,  -msg.axes[1])

		#set_speed(motor_left_ID,  (-msg.axes[1]+msg.axes[0])*scale)
		#set_speed(motor_right_ID,  (-msg.axes[1]-msg.axes[0])*scale)
		if (msg.axes[7]==1): #foward
			set_speed(motor_left_ID,-0.3)
			set_speed(motor_right_ID,-0.3)
			rospy.loginfo("forward")
		elif(msg.axes[7]==-1): #backward
			set_speed(motor_left_ID,0.3)
			set_speed(motor_right_ID,0.3)
			rospy.loginfo("backward")
		elif(msg.axes[6]==-1): # right
			set_speed(motor_left_ID,-0.5)
			set_speed(motor_right_ID,-0.3)
			rospy.loginfo("right")
		elif(msg.axes[6]==1): #left
			set_speed(motor_left_ID,-0.3)
			set_speed(motor_right_ID,-0.5)
			rospy.loginfo("left")
		else:
			set_speed(motor_left_ID,0)
			set_speed(motor_right_ID,0)
			rospy.loginfo("stop")

def on_cmd_mode(msg):
	global mode
	mode=msg.data
	rospy.loginfo("mode:%s",mode)
		

mode=''
# initialization
if __name__ == '__main__':

	# setup motor controller
	motor_driver = Adafruit_MotorHAT(i2c_bus=1)

	motor_left_ID = 1
	motor_right_ID = 2

	motor_left = motor_driver.getMotor(motor_left_ID)
	motor_right = motor_driver.getMotor(motor_right_ID)

	# stop the motors as precaution
	all_stop()

	# setup ros node
	rospy.init_node('jetbot_teleop')
	
	rospy.Subscriber('/joy', Joy, on_cmd_joy)
	rospy.Subscriber('/mode',String, on_cmd_mode)

	#rospy.Subscriber('~cmd_str', String, on_cmd_str)

	# start running
	rospy.spin()

	# stop motors before exiting
	all_stop()

