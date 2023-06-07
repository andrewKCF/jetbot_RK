#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64 #contro_effort
from sensor_msgs.msg import Joy
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
Motor_left.run(Adafruit_MotorHAT.FORWARD)
Motor_right.run(Adafruit_MotorHAT.FORWARD)
auto_mode=0

def on_cmd_joy(msg):
	#left +1, right -1 , up +1 , down -1
	global auto_mode
	auto_mode=msg.buttons[0]
	#rospy.loginfo("auto mode:%d",auto_mode)

def on_pid_effort(data):
	global mode
	speed=30
	#pwm=abs(int(data.data)-speed)
	pwm=abs(speed-int(data.data))
	#rospy.loginfo("contro_effort %f  left(fix):%d  right:%d",data.data,speed,pwm)	

	if (mode=='auto'):
		Motor_right.setSpeed(pwm)
		Motor_left.setSpeed(speed)
		rospy.loginfo("pid controller on")

mode=''
time_start=0
time_end=0
time_c=0
def on_cmd_mode(msg):
	global mode
	global time_start,time_end,time_c
	mode=msg.data


	
def listener():
	rospy.init_node('motor_lane', anonymous=True)
	rospy.Subscriber("control_effort", Float64, on_pid_effort)
	rospy.Subscriber('/joy', Joy, on_cmd_joy)
	rospy.Subscriber('/mode',String, on_cmd_mode)
	pub_mode = rospy.Publisher('mode', String, queue_size=1)
	#rospy.spin()
	rate=rospy.Rate(10)
	cmd_mode=String()
	while not rospy.is_shutdown():
		try:
			cmd_mode=rospy.wait_for_message('/mode',String,timeout=1)
		except:
			cmd_mode.data=None
			pass
		rospy.loginfo("recive:%s",cmd_mode.data)
		if (cmd_mode.data=="left_turn"):
			Motor_right.setSpeed(32)
			Motor_left.setSpeed(10)
			time.sleep(2)
			rospy.loginfo("left_turn")
			pub_mode.publish('auto')

		if (cmd_mode.data=="right_turn"):
			Motor_right.setSpeed(10)
			Motor_left.setSpeed(30)
			time.sleep(2)
			rospy.loginfo("right_turn")
			pub_mode.publish('auto')

		rate.sleep()

if __name__ == '__main__':
    listener()
