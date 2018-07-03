#!/usr/bin/env python
import numpy as np
import cv2
import csv
                            
camcalibvals = np.load("cam_calibration_values.npz")
cameraMatrix = camcalibvals['cameraMatrix']
distCoeffs = camcalibvals['distCoeffs']
camcalibvals.close()

objectPoints = []
imagePoints = []

objp1 = np.array([-11.,0.,0.],np.float32)
objp2 = np.array([11.,0.,0.],np.float32)
objp3 = np.array([-11.,18.,0.],np.float32)
objp4 = np.array([11.,18.,0.],np.float32)
objectPoints = np.array([objp1,objp2,objp3,objp4],np.float32)

imp1 = np.array([219.,720.],np.float32)
imp2 = np.array([1081.,720.],np.float32)
imp3 = np.array([380.,361.],np.float32)
imp4 = np.array([906.,360.],np.float32)
imagePoints = np.array([imp1,imp2,imp3,imp4],np.float32)

retval,rvec,tvec = cv2.solvePnP(objectPoints,imagePoints,cameraMatrix,distCoeffs)

rotationMatrix,jacobian=cv2.Rodrigues(rvec)

rtMatrix = np.append(rotationMatrix,tvec,1)

print("(0,3)")
uvPoint = np.array([np.array([649.],np.float32),
		np.array([613.],np.float32),
		np.array([1],np.float32)],np.float32)

#s*uvPoint = cameraMatrix.dot(rtMatrix).dot(xyzPoint)
camMatInv = np.linalg.inv(cameraMatrix)
rotMatInv = np.linalg.inv(rotationMatrix)
s = 0 + rotMatInv.dot(tvec)[2][0]
s /= rotMatInv.dot(camMatInv).dot(uvPoint)[2][0]

xyzPoint = rotMatInv.dot(camMatInv.dot(s*uvPoint)-tvec)
print(round(xyzPoint[0],3))
print(round(xyzPoint[1],3))
print(round(xyzPoint[2],3))


