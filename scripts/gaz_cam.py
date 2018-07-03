#!/usr/bin/env python

#import the necessary packages
import numpy
import sys
import cv2
from cv_bridge import CvBridge, CvBridgeError
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Int16MultiArray

class image_converter:
	def __init__(self):
		self.bridge = CvBridge()
		self.coord_pub = rospy.Publisher("bottom_pixels",Int16MultiArray,queue_size=5)
		self.image_sub = rospy.Subscriber("pioneer_camera/image_raw", Image, self.callback)


	def callback(self,data):
		try:
			cv_image=self.bridge.imgmsg_to_cv2(data,"bgr8")
		except CvBridgeError as e:
			rospy.loginfo(e)
		self.obj_detection(cv_image)
			
	def obj_detection(self, cv_image):
		wood_lower = numpy.array([10,60,60])
		wood_upper = numpy.array([100,255,255])
		
		blurred=cv2.GaussianBlur(cv_image,(11,11),0)
		hsv = cv2.cvtColor(blurred,cv2.COLOR_BGR2HSV)
		mask = cv2.inRange(hsv,wood_lower,wood_upper)
		mask=cv2.erode(mask,None,iterations=2)
		mask = cv2.dilate(mask,None, iterations =2)
		_, contours, _ = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)
		cv2.drawContours(cv_image,contours,-1,(0,180,180),2)
		cv2.imshow("!!!",cv_image)
		cv2.waitKey(5)
		
		self.findBotPixs(contours)
				
	def findBotPixs(self,contours):
		pixl_list = [[0,0]]
		if not contours:
			return(pixl_list[0])
		for pixel in contours:
			for ycord in pixel:
				if pixl_list[0][1]<ycord[0][1]:
					pixl_list = [[ycord[0][0],ycord[0][1]]]
				elif pixl_list[0][1]==ycord[0][1]:
					pixl_list.append(ycord[0])
		self.oneDimenArray(pixl_list)	
		
	#turn into a 1D array
	def oneDimenArray(self,array):
		one_d = []
		for pixl in array:
			for num in pixl:
				one_d.append(num)
		self.coord_pub.publish(Int16MultiArray(data=one_d))
	
def main(args):
	ic = image_converter()
	rospy.init_node('image_converter', anonymous=True)
	rospy.spin()

	
if __name__ == '__main__':
	main(sys.argv)
