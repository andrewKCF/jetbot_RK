#!/usr/bin/env python
# -*- coding: utf-8 -*
import roslib;
import rospy
 
from sensor_msgs.msg import Joy
from std_msgs.msg import String
 
class Teleop:
    def __init__(self):
        rospy.init_node('teleop_joy')
 
        self.cmd = None
        cmd_pub = rospy.Publisher('/motors/cmd_str', String,queue_size=10)
 
        rospy.Subscriber("joy", Joy, self.callback)
        rate = rospy.Rate(rospy.get_param('~hz', 20))
        
        while not rospy.is_shutdown():
            rate.sleep()
            if self.cmd:
                cmd_pub.publish(self.cmd)
 
    def callback(self, data):
        """ Receive joystick data, formulate String message. """
        self.cmd="stop"

        if data.axes[7] == 1:
            self.cmd="backward"
        elif data.axes[7] == -1:
            self.cmd="forward"

        if data.axes[6] == 1:
            self.cmd="right"
        elif data.axes[6] == -1:
            self.cmd="left"

	'''            
        if data.buttons[0] == 1:
            self.cmd = cmd
            rospy.loginfo(cmd)
        else:
            self.cmd = None
	'''

if __name__ == "__main__": Teleop()

