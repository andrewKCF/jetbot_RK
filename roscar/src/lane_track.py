#!/usr/bin/env python
#!coding=utf-8
import numpy as np
import cv2

import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image # subscribe image
from std_msgs.msg import Float64 # setpoint state # contro_effort
from std_msgs.msg import Bool # setpoint state
from std_msgs.msg import String # setpoint state
from sensor_msgs.msg import Joy
from vision_msgs.msg import Detection2D, Detection2DArray


state=0
image_height=0
image_width=0
box_size_x=0
box_size_y=0
box_center_x=0
box_center_y=0


def callback(data):
	# define picture to_down' coefficient of ratio
	global bridge
	global state,image_height,image_width
	global pub_enable
	global mode
	global st
	global box_size_x,box_size_y,box_center_x,box_center_y
	img = bridge.imgmsg_to_cv2(data, "bgr8")
	blur = cv2.GaussianBlur(img,(5,5),0)
	hsv = cv2.cvtColor(blur,cv2.COLOR_BGR2HSV)

	low_lane = np.array([ 0, 59,  0])
	high_lane = np.array([ 42, 255, 255])
	mask = cv2.inRange(hsv,low_lane,high_lane)
	kernel = np.ones((5,5), np.uint8)
	mask = cv2.erode(mask, kernel, iterations = 2)
	isolate=np.zeros_like(mask)
	rectangle=np.array([[(0,int(image_height*0.6)),(image_width,int(image_height*0.6)),(image_width,image_height),(0,image_height)]])
	isolate=cv2.fillPoly(isolate,rectangle,255)
	mask=cv2.bitwise_and(mask,isolate)
	
	canny = cv2.Canny(mask,20,100)
    #lines = cv2.HoughLinesP(canny,1,np.pi/180,50,maxLineGap=50,minLineLength=20)
	lines = cv2.HoughLinesP(canny,1,np.pi/120,10,maxLineGap=30,minLineLength=20)
	#cv2.imshow("maks", mask)
	#cv2.imshow("canny", canny)
    #if not ('lines' in locals()):
	if lines is None:
		#state=image_width
		#pub_enable.publish(False)
		#cv2.imshow("lane_track" , mask)
		cv2.putText(img, mode, (image_width/2,image_height-100), cv2.FONT_HERSHEY_SIMPLEX,  1, (0, 255, 255), 1, cv2.LINE_AA)	
		# detection
		cv2.rectangle(img,(int(box_center_x-box_size_x/2),int(box_center_y-box_size_y/2)),
		(int(box_center_x+box_size_x/2),int(box_center_y+box_size_y/2)),(0,0,255),3,cv2.LINE_AA)
		# detection --END
		cv2.imshow("lane_track", img)
		cv2.waitKey(1)
		return
	#print(lines.shape)
	for line in lines:
		#pub_enable.publish(True)
		x1,y1,x2,y2 = line[0]
		cv2.circle(img,(x1,y1), 10, (0, 0, 255), 3)
		cv2.circle(img,(x2,y2), 2, (0, 0, 255), 3)
		cv2.line(img,(x1,y1),(x2,y2),(255,0,0),3)
		#print('x1:{:.2f}  y1:{:.2f}  x2:{:.2f}  y2:{:.2f}'.format(x1,y1,x2,y2))
		dy=y2-y1
		dx=x2-x1
		m=float(dy)/float(dx)
		rospy.loginfo("m:%f",m)
		print("dx:%d dy:%d m:%f"%(dx,dy,m))
		if (m) > 0 or (m>-0.7): #ignor right side lane
			break
		fx=(image_height-y1)/m+x1
		fx=int(fx)
		cv2.circle(img,(image_width/2,image_height), 10, (0, 255, 255), 3) # center point
		cv2.circle(img,(fx,image_height), 10, (0, 255, 255), 3) # find point
		cv2.line(img,(fx,image_height),(image_width/2,image_height),(255,0,0),3)
		if (x1<image_width/2):
			#state=st-fx
			state=image_width/2-fx	
			text="state:"+str(state)
			cv2.putText(img, text, (image_width/2,image_height-20), cv2.FONT_HERSHEY_SIMPLEX,  1, (0, 255, 255), 1, cv2.LINE_AA)
			cv2.putText(img, "setpoint:"+str(st), (image_width/2,image_height-60), cv2.FONT_HERSHEY_SIMPLEX,  1, (0, 255, 255), 1, cv2.LINE_AA)	
			#print('state:{:.2f}   dy/dx:{:.2f}'.format(state,dy/dx))
			break
	cv2.putText(img, mode, (image_width/2,image_height-100), cv2.FONT_HERSHEY_SIMPLEX,  1, (0, 255, 255), 1, cv2.LINE_AA)	
	# detection
	cv2.rectangle(img,(int(box_center_x-box_size_x/2),int(box_center_y-box_size_y/2)),
	(int(box_center_x+box_size_x/2),int(box_center_y+box_size_y/2)),(0,0,255),3,cv2.LINE_AA)
	# detection --END
	cv2.imshow("lane_track", img)
	cv2.waitKey(1)



def detectCB(data):
	pub_detect = rospy.Publisher('sign_detect', String, queue_size=1)	
	print('----------callback_func-------------------')
	global box_size_x,box_size_y,box_center_x,box_center_y
	box_size_x=0
	box_size_y=0
	box_center_x=0
	box_center_y=0
	for i in xrange(len(data.detections)):
		#rospy.loginfo('index:%d',i)
		rospy.loginfo('id:%s',str(data.detections[i].results[0].id))
		#rospy.loginfo('score:%s',str(data.detections[i].results[0].score))
		#rospy.loginfo('center:%s',type(data.detections[i].bbox.center.x))
		#rospy.loginfo('size_x:%s',str(data.detections[i].bbox.size_x))
		#rospy.loginfo('size_y:%s',str(data.detections[i].bbox.size_y))
		if ((data.detections[i].results[0].id==1) and (data.detections[i].results[0].score>0.98)): #left
			pub_detect.publish('left_turn')
			box_center_x=int(data.detections[i].bbox.center.x)
			box_center_y=int(data.detections[i].bbox.center.y)
			box_size_x=int(data.detections[i].bbox.size_x)
			box_size_y=int(data.detections[i].bbox.size_y)
		if ((data.detections[i].results[0].id==2) and (data.detections[i].results[0].score>0.90)): #right
			pub_detect.publish('right_turn')
			box_center_x=int(data.detections[i].bbox.center.x)
			box_center_y=int(data.detections[i].bbox.center.y)
			box_size_x=int(data.detections[i].bbox.size_x)
			box_size_y=int(data.detections[i].bbox.size_y)
	#print('score:'+str(data.detections[0].results[0].score))
	#print(type(data.detections)) # list
	#print(type(data.detections[0])) # class
	#print(type(data.detections[0].results)) # list
	#print(type(data.detections[0].bbox)) #class


mode='manual'
def on_cmd_mode(msg):
	global mode
	mode=msg.data
	#rospy.loginfo("on cmd CB auto mode:%s",mode)

def lane_track():
	rospy.init_node('lane_track', anonymous=True)
	# make a video_object and init the video object
	global bridge,state
	global pub_enable # pid enable
	global st # pid set point
	bridge = CvBridge()
	bridge_det=CvBridge()
	rospy.Subscriber('/mode',String, on_cmd_mode)
	rospy.Subscriber('/video_source/raw', Image, callback)
	rospy.Subscriber('/detectnet/detections',Detection2DArray,detectCB)
	rate = rospy.Rate(10) # 10hz
	pub_state = rospy.Publisher('state', Float64, queue_size=10)
	pub_setpoint = rospy.Publisher('setpoint', Float64, queue_size=10)
	#pub_enable = rospy.Publisher('pid_enable', Bool, queue_size=10)
	#rospy.Subscriber('/joy', Joy, on_cmd_joy)
	#st=image_width/2
	#st=240  #first
	st=300  #first
	while not rospy.is_shutdown():
		#pub_enable.publish(True)
		pub_setpoint.publish(st)
		pub_state.publish(state)
		str="setpoint %3.2f"%st + ",  state %3.2f"%state
		rospy.loginfo(str)
		'''
		rospy.loginfo("auto mode:%d",auto_mode)
		if (auto_mode==1):
			rospy.loginfo("controller on")
			pub_state.publish(state)
		else:
			pub_state.publish(image_width/2)
			rospy.loginfo("controller off")
		'''

        #print("state:%5f",state)
		rate.sleep()

	rospy.spin()	

if __name__ == '__main__':
	image_height=rospy.get_param('/video_source/height')
	image_width=rospy.get_param('/video_source/width')
	#print("image height:%d width:%d",image_height,image_width)
	lane_track()

