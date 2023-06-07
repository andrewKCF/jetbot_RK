#!/usr/bin/env python
import sys
import rospy
import cv2
from vision_msgs.msg import Detection2D, Detection2DArray
from sensor_msgs.msg import Image # subscribe image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String

def callback(data):
	pub_detect = rospy.Publisher('sign_detect', String, queue_size=1)	
	print('----------callback_func-------------------')
	for i in xrange(len(data.detections)):
		#rospy.loginfo('index:%d',i)
		rospy.loginfo('id:%s',str(data.detections[i].results[0].id))
		#rospy.loginfo('score:%s',str(data.detections[i].results[0].score))
		#rospy.loginfo('center:%s',str(data.detections[i].bbox.center))
		#rospy.loginfo('size_x:%s',str(data.detections[i].bbox.size_x))
		#rospy.loginfo('size_y:%s',str(data.detections[i].bbox.size_y))
		if ((data.detections[i].results[0].id==1) and (data.detections[i].results[0].score>0.98)): #left
			pub_detect.publish('left_turn')
		if ((data.detections[i].results[0].id==2) and (data.detections[i].results[0].score>0.90)): #right
			pub_detect.publish('right_turn')
	#print('score:'+str(data.detections[0].results[0].score))
	#print(type(data.detections)) # list
	#print(type(data.detections[0])) # class
	#print(type(data.detections[0].results)) # list
	#print(type(data.detections[0].bbox)) #class

def detectcb(data):
	global bridge_det
	brideg_det=CvBridge()
	img = bridge_det.imgmsg_to_cv2(data, "bgr8")
	cv2.imshow("sign_detect", img)
	cv2.waitKey(1)


def main(args):
	#rospy.init_node('detector_node',anonymous=True)
	rospy.init_node('detector_node')
	rospy.Subscriber('/detectnet/detections',Detection2DArray,callback)
	rospy.Subscriber('/detectnet/overlay', Image, detectcb)
	global bridge_det
	bridge_det=CvBridge()
	
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("ShutDown")

if __name__=='__main__':
	main(sys.argv)
