#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
def callback(data):
	cv_img = bridge.imgmsg_to_cv2(data, "bgr8")
	cv2.imshow("webcam" , cv_img)
	cv2.waitKey(1)

def webcam_display():
    rospy.init_node('webcam_display', anonymous=True)

    # make a video_object and init the video object
    global count,bridge
    count = 0
    bridge = CvBridge()
    rospy.Subscriber('video_source/raw', Image, callback)
    rospy.spin()

if __name__ == '__main__':
    webcam_display()
