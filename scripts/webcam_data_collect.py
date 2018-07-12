#!/usr/bin/env python

import numpy as np
import cv2
import csv

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
	np.array([5.,673.],np.float32),
	np.array([1258.,667.],np.float32),
	np.array([278.,226.],np.float32),
	np.array([977.,216.],np.float32),
	np.array([1079.,375.],np.float32),
	np.array([178.,385.],np.float32),
	np.array([623.,375.],np.float32),
	np.array([622.,223.],np.float32),
	np.array([623,672.],np.float32)],np.float32)
			
retval,rvec,tvec = cv2.solvePnP(objectPoints,imagePoints,cameraMatrix,distCoeffs)
rotationMatrix,jacobian=cv2.Rodrigues(rvec)


file = open('webcam_data_adj','a')
filewriter = csv.writer(file,delimiter=',',quotechar='|',quoting=csv.QUOTE_MINIMAL)

#define the lower and upper boundaries of the wood in the HSV color space
    #Finding HSV values to track:
        #green = numpy.uint8([[[0,255,0]]])
        #hsv_green = cv2.cvtColor(green,cv2.COLOR_BGR2HSV)
        #print hsv_green
        #[H-10,100,100] and [H+10,255,255]
#purple
wood_lower = np.array([125,60,50])
wood_upper = np.array([175,255,255])
#blue
#wood_lower = np.array([95,60,60])
#wood_upper = np.array([120,255,255])


#grab reference to webcam front(0) back(1)
webcam = cv2.VideoCapture(1)
#change resolution to 1280x720
webcam.set(3,1280)
webcam.set(4,720)

#grab the current frame
grabbed, frame0 = webcam.read()
#undistort
h,w = frame0.shape[:2]
print((h,w))
newCameraMatrix, roi = cv2.getOptimalNewCameraMatrix(cameraMatrix,distCoeffs,(w,h),1,(w,h))
frame = cv2.undistort(frame0,cameraMatrix,distCoeffs,None,newCameraMatrix)

#print(frame.shape)
i=0
#while a frame is grabbed
while grabbed:
    if i==0:
        x_real = input("x:")
        y_real = input("y:")
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

    print([x_real,y_real,xyzPoint[0][0],xyzPoint[1][0]])    
    if not -2<=x_real-xyzPoint[0][0]<=2:
    	print("Not match")
    	if i == 0:
        	for i in range(5):
        		webcam.read()
    elif not -2<=y_real-xyzPoint[1][0]<=2:
    	print("Not match")
    	if i == 0:
        	for i in range(5):
        		webcam.read()
    else:
	    filewriter.writerow([x_real,y_real,0,bot_pix[0],bot_pix[1],xyzPoint[0][0],xyzPoint[1][0],xyzPoint[2][0]])
	    i+=1
	    if i==500:
		i=0
	    
    cv2.waitKey(25)
    
    cv2.imshow("frame",frame)
    #grab the next "current" frame
    grabbed, frame = webcam.read()
    
    #undistort
cv2.destroyAllWindows()
webcam.release()

