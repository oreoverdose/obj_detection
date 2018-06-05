#!/usr/bin/env python

#import the necessary packages
import numpy
import cv2
import cv_bridge
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import Image
import rospy

#height of camera
h_cam = rospy.get_param("h_cam")
b_length = rospy.get_param("b_length")
f_length = rospy.get_param("f_length")
asp_rat = rospy.get_param("asp_rat")
#horizontal angle of view, calculated
horiz_ang_view = numpy.arctan(f_length/2/b_length)*2
#vertical angle of view, calculated
vert_ang_view = numpy.arcsin(f_length/2*asp_rat/numpy.sqrt(b_length*b_length+h_cam*h_cam))*2
#pitch angle
pitch_ang = numpy.arctan(b_length/h_cam) + vert_ang_view/2
#height of object
h_obj = rospy.get_param("h_obj")
#diameter of object
d_obj = rospy.get_param("d_obj")
h_obj_cam = h_obj/2
h = h_cam - h_obj_cam + (h_obj/2)

	
#find the bottom and top pixel from contour
def findBotPixel (contours):
	if not contours:
		return([0,0])
	else:
		bottom_pixel = [0,0]
	for pixel in contours:
		for ycord in pixel:
			if bottom_pixel[1]<ycord[0][1]:
				bottom_pixel = ycord[0]	
	return (bottom_pixel)

#find (x*,y*) 
def webCamPosition (pixel):
	y_cam_pos = float(pixel[0])/640*22 #measured 22cm
	x_cam_pos = 16.5-float(pixel[1])/480*16.5 #measured 16.5cm
	return [x_cam_pos,y_cam_pos]

#find x position of object (forward backwards)
def findX(x_cam, y_cam):
     return ((x_cam * numpy.cos(pitch_ang) +(h*numpy.tan(pitch_ang-vert_ang_view/2)))*(h/(h-x_cam*numpy.sin(pitch_ang)))+d_obj/2)
#find y position of object (left right)
def findY(x_cam,y_cam):
     return ((y_cam-h*numpy.tan(pitch_ang-vert_ang_view/2)*numpy.tan(horiz_ang_view/2))*h/(h-x_cam*numpy.sin(pitch_ang)))

def talker():
	#grab reference to webcam front(0) back(1)
	webcam = cv2.VideoCapture(0)
	#grab the current frame
	grabbed, frame = webcam.read()
	br = cv_bridge.CvBridge()
	#declares that your node is publishing to the obj_coord topic using the message type Float32MultiArray
	#queue_size argument limits the amount of queued messages is any subscribe is nt receiving them fast enough
	coord_pub = rospy.Publisher('obj_coord',Float32MultiArray,queue_size = 10)
	img_pub = rospy.Publisher('image',Image,queue_size = 10)
	pxl_pub = rospy.Publisher('pxls',Int32MultiArray,queue_size = 10)
	#tells rospy the name of your node
	#anonymous = True insures that your node has a unique name by adding random numbers to the end of the name
	rospy.init_node('obj_detect',anonymous = True)
	#define the lower and upper boundaries of the wood in the HSV color space
	    #Finding HSV values to track:
		#green = numpy.uint8([[[0,255,0]]])
		#hsv_green = cv2.cvtColor(green,cv2.COLOR_BGR2HSV)
		#print hsv_green
		#[H-10,100,100] and [H+10,255,255]
	#purple
	wood_lower = numpy.array([125,60,50])
	wood_upper = numpy.array([175,255,255])
	#blue
	#wood_lower = numpy.array([95,60,60])
	#wood_upper = numpy.array([120,255,255])
	
	#print(frame.shape)
	while not rospy.is_shutdown():
		#blur
		blurred=cv2.GaussianBlur(frame,(11,11),0)
	    #convert to the HSV color space
		hsv = cv2.cvtColor(blurred,cv2.COLOR_BGR2HSV)
		    #construct a mask for the color "wood"
		mask = cv2.inRange(hsv,wood_lower,wood_upper)
	    #perform a series of dilations and erions to remove any small blobs
	    #left in the mask
		mask=cv2.erode(mask,None,iterations=2)
		mask = cv2.dilate(mask,None, iterations =2)
		    #find contours in the mask (????????????)
	    #findContours returns 3 values, returns the boundaries of the white mask
		_, contours, _ = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)
		    #draw the contours on the frame. -1 draws all of them.
		cv2.drawContours(frame,contours,-1,(0,180,180),2)
		cv2.drawContours(blurred,contours,-1,(0,180,180),2)
		cv2.imshow("frame",frame)
		
	    #convert cv::Mat into a ros image message, CvBridge provides the following function
		image_message = br.cv2_to_imgmsg(frame,encoding='passthrough')
		img_pub.publish(image_message)
		#rospy.loginfo(image_message)
		
		bot_pix = findBotPixel(contours)
		pxl_pub.publish(data=[bot_pix])
		

	   
	   #finding (x*,y*) of bottom pixel
		x_cam,y_cam = webCamPosition(bot_pix)
	    	#print(x_cam,y_cam)
	    
	     
		x = findX(x_cam, y_cam)
		y = findY(x_cam, y_cam)
			
		coord_vect = Float32MultiArray(data=[x,y,h_obj_cam/2])
		#rospy.loginfo(coord_vect)
		coord_pub.publish(coord_vect)
		#print(x,y)
		    #wait 25miliseconds  
		key = cv2.waitKey(25)
		if key == 27:#exit on ESC
			break     
	    #grab the next "current" frame
		grabbed, frame = webcam.read()
	webcam.release()
if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass
