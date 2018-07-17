#!/usr/bin/env python

#import the necessary packages
import numpy as np
import sys
import cv2
import rospy
from geometry_msgs.msg import Point

class image_converter:
	def __init__(self):
		self.coord_pub = rospy.Publisher("bottom_pixels",Int16MultiArray,queue_size=1)
		
		cameraMatrix = np.array([
			np.array([1328.51479,0.,601.225024],np.float64),
			np.array([0.,1327.02605,337.524817],np.float64),
			np.array([0.,0.,1],np.float64)],np.float64)
		distCoeffs = np.array([np.array([0.07580839,-0.24187201,-0.00365459,-0.0003942,0.10928811],np.float64)],np.float64)
	
		objectPoints = 2*np.array([
			np.array([-13.5,0.,0.],np.float32),
			np.array([13.5,0.,0.],np.float32),
			np.array([-13.5,22,0.],np.float32),
			np.array([13.5,22,0.],np.float32),
			np.array([13.5,11.,0.],np.float32),
			np.array([-13.5,11.,0.],np.float32),
			np.array([0,11,0.],np.float32),
			np.array([0,22,0.],np.float32),
			np.array([0,0,0.],np.float32)],np.float32)
	
		imagePoints = np.array([
			np.array([82.,625.],np.float32),
			np.array([1247.,622.],np.float32),
			np.array([301.,149.],np.float32),
			np.array([1002.,133.],np.float32),
			np.array([1097.,308.],np.float32),
			np.array([215.,324.],np.float32),
			np.array([646.,320.],np.float32),
			np.array([645.,140.],np.float32),
			np.array([650,622.],np.float32)],np.float32)
			
		retval,rvec,tvec = cv2.solvePnP(objectPoints,imagePoints,cameraMatrix,distCoeffs)
		rotationMatrix,jacobian=cv2.Rodrigues(rvec)
		
		#red
		wood_lower = np.array([0,180,50])
		wood_upper = np.array([190,255,255])
		#purple
		#wood_lower = np.array([125,60,50])
		#wood_upper = np.array([175,255,255])
		#blue
		#wood_lower = np.array([95,60,60])
		#wood_upper = np.array([120,255,255])

			
	def findBotPixel (contours):
	     pixl_list = [[0,0]]
	     if not contours:
		  bottom_pixel = [0,0]
	     else:
		  bottom_pixel = [0,0]
	     for pixel in contours:
		for ycord in pixel:
			if pixl_list[0][1]<ycord[0][1]:
				pixl_list = [[ycord[0][0],ycord[0][1]]]
			elif pixl_list[0][1]==ycord[0][1]:
				pixl_list.append(ycord[0])
	     bottom_pixel = pixl_list[int(len(pixl_list)/2)]
	     return(bottom_pixel)
		  
	def getLocalObjPos(pix):
		#pixel position turns into uvPoint
		uvPoint = np.array([
			np.array([pix[0]],np.float32),
			np.array([pix[1]],np.float32),
			np.array([1],np.float32)],np.float32)
		#s*uvPoint = cameraMatrix.dot(rtMatrix).dot(xyzPoint)
		#solve for s
		camMatInv = np.linalg.inv(cameraMatrix)
		rotMatInv = np.linalg.inv(rotationMatrix)
		s = 0 + rotMatInv.dot(tvec)[2][0]
		s /= rotMatInv.dot(camMatInv).dot(uvPoint)[2][0]
		
		xyzPoint = rotMatInv.dot(camMatInv.dot(s*uvPoint)-tvec)
		#publish local posiiton
		return(xyzPoint)
	
def main(args):
	ic = image_converter()
	rospy.init_node('cam_localizer', anonymous=True)
	
	#grab reference to webcam front(0) back(1)
	webcam = cv2.VideoCapture(1)
	#change resolution to 1280x720
	webcam.set(3,1280)
	webcam.set(4,720)

	#grab the current frame
	grabbed, frame0 = webcam.read()
	h,w = frame0.shape[:2]
	print((h,w))
	
	while grabbed:
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

	    bot_pix = findBotPixel(contours)
	    #finding (x*,y*) of bottom pixel
	    xyzPoint = getLocalObjPos(bot_pix)

	
if __name__ == '__main__':
	main(sys.argv)
