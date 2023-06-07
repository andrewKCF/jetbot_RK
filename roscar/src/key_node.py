#!/usr/bin/env python
import getch
import rospy
from std_msgs.msg import String #String message 
from std_msgs.msg import Int8


################################
# created by yuvaram
#yuvaramsingh94@gmail.com
################################


def keys():
	pub = rospy.Publisher('key',Int8,queue_size=10) # "key" is the publisher name
	rospy.init_node('key',anonymous=True)
	rate = rospy.Rate(10)#try removing this line ans see what happens
	while not rospy.is_shutdown():
		k=ord(getch.getch())# this is used to convert the keypress event in the keyboard or joypad , joystick to a ord value
		if ((k>=65)&(k<=68)|(k==109)|(k==97)|(k==113)):# to filter only the up , dowm ,left , right key /// this line can be removed or more key can be added to this
			# forwad:65 backward:66 right:67 left:68 
			# a:97 m:109 q:113
			if (k==65):
				rospy.loginfo("forward")# to print on  terminal
			if (k==66):
				rospy.loginfo("backward")# to print on  terminal
			if (k==67):
				rospy.loginfo("right_turn")# to print on  terminal
			if (k==68):
				rospy.loginfo("left_turn")# to print on  terminal
			if (k==109):
				rospy.loginfo("manual")# to print on  terminal
			if (k==97):
				rospy.loginfo("auto")# to print on  terminal
			if (k==113):
				rospy.loginfo("quit")# to print on  terminal

			pub.publish(k)#to publish
		if (k==113): # quit
			return
if __name__=='__main__':
	try:
		keys()
	except rospy.ROSInterruptException:
		pass
