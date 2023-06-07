#!/usr/bin/env python
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image # subscribe image
from std_msgs.msg import Float64 # setpoint state # contro_effort
from std_msgs.msg import Bool # setpoint state
from sensor_msgs.msg import Joy
import numpy as np
import cv2

midx=0
image_height=0
image_width=0


def on_cmd_joy(msg):
	#left +1, right -1 , up +1 , down -1
	global auto_mode
	auto_mode=msg.buttons[0]
	#rospy.loginfo("auto mode:%d",auto_mode)


def callback(data):
	# define picture to_down' coefficient of ratio
	global bridge
	global midx,image_height,image_width


	img = bridge.imgmsg_to_cv2(data, "bgr8")

    #define kernel size  
	kernel = np.ones((7,7),np.uint8)
    # convert to hsv colorspace 
    # img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    # lower bound and upper bound
	lower_bound = np.array([ 50, 127,   0])     
	upper_bound = np.array([104, 255,  72])
    # find the colors within the boundaries
	mask = cv2.inRange(img, lower_bound, upper_bound)
    # Remove unnecessary noise from mask
    #mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    #mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    # Segment only the detected region
	segmented_img = cv2.bitwise_and(img, img, mask=mask)
    # Find contours from the mask
    #_,contours, hierarchy = cv2.findContours(mask.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
	contours, hierarchy = cv2.findContours(mask.copy(), cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

    # Find the index of the largest contour
	areas=[cv2.contourArea(c) for c in contours]
	if not areas:
		midx=image_width/2
		cv2.imshow("color_track",img)
		cv2.waitKey(1)
		return
       

	max_index=np.argmax(areas)
	cnt=contours[max_index]
    # Draw boundary box
	x,y,w,h=cv2.boundingRect(cnt)
	cv2.rectangle(img,(x,y),(x+w,y+h),(0,255,0),2)
	midx=x+w/2
	midy=y+h/2
	cv2.circle(img,(midx,midy),2,(0,0,255),4)
    
    
	cv2.imshow("color_track" , img)
	cv2.waitKey(1)

def color_tracker():
	rospy.init_node('color_tracker', anonymous=True)
	# make a video_object and init the video object
	global bridge,midx
	global auto_mode
	auto_mode=0
	bridge = CvBridge()
	rospy.Subscriber('/video_source/raw', Image, callback)
	rospy.Subscriber('/joy', Joy, on_cmd_joy)
	rate = rospy.Rate(10) # 10hz
	pub_state = rospy.Publisher('state', Float64, queue_size=10)
	pub_setpoint = rospy.Publisher('setpoint', Float64, queue_size=10)
	pub_enable = rospy.Publisher('pid_enable', Bool, queue_size=10)

	while not rospy.is_shutdown():
		pub_enable.publish(True)
		pub_setpoint.publish(image_width/2)
		print(midx)
		#print(auto_mode)
		rospy.loginfo("auto mode:%d",auto_mode)
		if (auto_mode==1):
			rospy.loginfo("controller on")
			pub_state.publish(midx)
		else:
			pub_state.publish(image_width/2)
			rospy.loginfo("controller off")
		rate.sleep()

	rospy.spin()

if __name__ == '__main__':
	image_height=rospy.get_param('/video_source/height')
	image_width=rospy.get_param('/video_source/width')
	color_tracker()
