#!/usr/bin/env python
#!coding=utf-8

#right code !
#write by leo at 2018.04.26
#function: 
#display the frame from another node.

import numpy as np
import cv2
face_cascade = cv2.CascadeClassifier('/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml')

import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Float64 # setpoint state # contro_effort
from std_msgs.msg import Bool # setpoint state


midx=0
image_height=0
image_width=0


def callback(data):
    
    #scaling_factor = 0.5
    global bridge,midx,face_cascade,image_height,image_width
    img = bridge.imgmsg_to_cv2(data, "bgr8")
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    faces = face_cascade.detectMultiScale(gray, 1.1, 4)

    if len(faces)==0: #check tube
	midx=image_width/2
        cv2.imshow("face_track" , img)
        cv2.waitKey(1)
	return

    for (x, y, w, h) in faces:
        cv2.rectangle(img, (x, y), (x+w, y+h), (255, 0, 0), 2)
	midx=x+w/2


    cv2.imshow("face_track" ,img)
    cv2.waitKey(1)

def face_track():
    rospy.init_node('face_track', anonymous=True)

    # make a video_object and init the video object
    global bridge,midx
    bridge = CvBridge()
    rospy.Subscriber('/video_source/raw', Image, callback,queue_size=1,buff_size=2**24)
    rate = rospy.Rate(10) # 10hz
    pub_enable = rospy.Publisher('pid_enable', Bool, queue_size=10)
    pub_setpoint = rospy.Publisher('setpoint', Float64, queue_size=10)
    pub_state = rospy.Publisher('state', Float64, queue_size=10)

    while not rospy.is_shutdown():
        pub_enable.publish(True)
        pub_setpoint.publish(120)
        print(midx)
        pub_state.publish(midx)
    	rate.sleep()

    rospy.spin()

if __name__ == '__main__':
    image_height=rospy.get_param('/video_source/height')
    image_width=rospy.get_param('/video_source/width')
    face_track()
