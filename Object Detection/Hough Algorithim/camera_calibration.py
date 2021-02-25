kkq# -*- coding: utf-8 -*-
"""
Created on Wed Feb 24 01:03:58 2021

@author: AWasf
"""

import numpy as np
import cv2
# import glob

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 20, 0.001)  # Changed here

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((9*7,3), np.float32)
objp[:,:2] = np.mgrid[0:7,0:9].T.reshape(-1,2)

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.
images = []    # Images captured which had the pattern 
n = 0
# images = glob.glob('*.jpg')
vid = cv2.VideoCapture(0) 


# for fname in images:
while n < 20:
    # img = cv2.imread(fname)
    ret, frame = vid.read()
    cv2.imshow('... Searching for the Calibration checkerboard ...',cv2.flip(frame,1))
    
    gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)

    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray, (7,9),None)      # Changed here
    
    print("****** Capture this frame ?! ")
    
    # If found, add object points, image points (after refining them)
    if ret == True and cv2.waitKey(1) & 0xFF == ord('k'):
        objpoints.append(objp)

        corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
        imgpoints.append(corners2)

        # Draw and display the corners
        frame = cv2.drawChessboardCorners(frame, (7,9), corners2,ret)
        images.append(frame)
        
        cv2.imshow('... Frame Captured ...',cv2.flip(frame,1))
        print("****** Frame Captured ******")
        
        n += 1
        print("Number of Captured Frames: ",n)

    if cv2.waitKey(1) & 0xFF == ord('q'): 
        break
        
vid.release() 
cv2.destroyAllWindows()

ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

np.save("objp",objp)
np.save("imgpoints",imgpoints)
np.save("images",images)
np.save("ret", ret)
np.save("mtx", mtx)
np.save("dist", dist)
np.save("rvecs", rvecs)
np.save("tvecs", tvecs)