#!/usr/bin/env python
#!coding=utf-8
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from std_msgs.msg import Int8

mode='manual'
def on_cmd_joy(msg):
	#left +1, right -1 , up +1 , down -1
	#auto_mode=msg.buttons[0]
	global mode
	pub_mode = rospy.Publisher('mode', String, queue_size=1)
	if (msg.buttons[0]==1):
		if (mode=='auto'):
			mode='manual'
		else:
			mode='auto'
	pub_mode.publish(mode)
	rospy.loginfo("(joy)mode:%s",mode)

def on_cmd_key(msg):
	global mode
	pub_mode = rospy.Publisher('mode', String, queue_size=1)
	# forwad:65 backward:66 right:67 left:68 
	# a:97 m:109 q:113
	if (msg.data==97):
		mode='auto'
	if (msg.data==109):
		mode='manual'
	pub_mode.publish(mode)
	rospy.loginfo("(key)mode:%s",mode)

def on_cmd_sign(msg):
	pub_mode = rospy.Publisher('mode', String, queue_size=1)

	if (mode=='auto'):
		pub_mode.publish(msg.data)
		

def mode_selection():
	global mode
	rospy.init_node('mode_selection', anonymous=True)
	rospy.Subscriber('/joy', Joy, on_cmd_joy)
	rospy.Subscriber('/key', Int8, on_cmd_key)
	rospy.Subscriber('/sign_detect', String, on_cmd_sign)
	'''
	rate = rospy.Rate(10) # 10hz
	#pub_mode = rospy.Publisher('mode', String, queue_size=1)
	while not rospy.is_shutdown():
		pub_mode.publish(mode)
		rospy.loginfo("mode:%s",mode)
		rate.sleep()
	'''

	rospy.spin()	

if __name__ == '__main__':
    mode_selection()

