# -*- coding: utf-8 -*-
"""
Created on Tue Mar  2 20:20:24 2021

@author: AWasf
"""

# all plots will be set directly below the code cell that produced it
# %matplotlib inline
from matplotlib import pyplot as plt
# from IPython.display import clear_output
# set inline plots size
plt.rcParams["figure.figsize"] = (16, 10) # (w, h)
# remove grid lines
import numpy as np
import time
import cv2
from math import sqrt

"""
        HELPER FUNCTIONS
"""
# funcrion to read and resize an image
def read_and_resize(filename, grayscale = False, fx= 0.5, fy=0.5):
    if grayscale:
      img_result = cv2.imread(filename, cv2.IMREAD_GRAYSCALE)
    else:
      imgbgr = cv2.imread(filename, cv2.IMREAD_COLOR)
      # convert to rgb
      img_result = cv2.cvtColor(imgbgr, cv2.COLOR_BGR2RGB)
    # resize
    img_result = cv2.resize(img_result, None, fx=0.5, fy=0.5, interpolation = cv2.INTER_CUBIC)
    return img_result

    
def showInRow(list_of_images, titles = None, disable_ticks = False):
  plt.rcParams["figure.figsize"] = (16, 10) # (w, h)
  count = len(list_of_images)
  for idx in range(count):
    subplot = plt.subplot(1, count, idx+1)
    if titles is not None:
      subplot.set_title(titles[idx])
      
    img = list_of_images[idx]
    cmap = 'gray' if (len(img.shape) == 2 or img.shape[2] == 1) else None
    subplot.imshow(img, cmap=cmap)
    #subplot.imshow(img, cmap=cmap,vmin=0, vmax=1) to disable normalization
    if disable_ticks:
      plt.xticks([]), plt.yticks([])
  plt.show()

 # function for colors array generation
def generate_colors(num):
  r = lambda: np.random.randint(0,255)
  return [(r(),r(),r()) for _ in range(num)] 

def process_video(video_path, frame_process):
  vid = cv2.VideoCapture(video_path)
  try:
    while(True):
      ret, frame = vid.read()
      if not ret:
        vid.release()
        break

#       frame = cv2.resize(frame, (int(frame.shape[1]), int(frame.shape[0])))        
#       frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

      if frame_process is not None:
        # frame = cv2.flip(frame,1)
        frame = frame_process(frame)
#         frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
#         out.write(frame)
        cv2.imshow("Video",frame)
        if cv2.waitKey(1) & 0xFF == ord('q'): 
            break
            
        if cv2.waitKey(1) & 0xFF == ord('c'): 
            cv2.imwrite("DetectedCylinder.jpg",frame)            

    vid.release()
    cv2.destroyAllWindows()
  
  except KeyboardInterrupt:
    vid.release()
    cv2.destroyAllWindows()
    

# def circledetect():

#     if add_par is None:
#         add_par=np.array([2,150])#resolution and hole separation
#     if zcord is None:
#        zcord=0
#     if msk is None:#incase the whole image is to be searched msk and msk_edge are None, i.e hole detection and object detection
#         msk=255*np.ones(frame.shape)
#         msk_edge=msk
#     if minmaxrang is None:
#         minmaxrang=np.array([minRadius,maxRadius])
#         minmaxrang=minmaxrang.astype('int')
#     if cannythresh is None:
#        cannythresh=np.array([cannythreshold1,cannythreshold2])   
#     cent_rad_mask=np.loadtxt(fcentrad)#center and radius of mask detected earlier      
#     mskimg=impreprocanny(frame,msk,msk_edge,cannythreshold1,cannythreshold2)
#     cv.imwrite('maskedframecannyfilt.png',mskimg) 
#     msk=msk.astype('uint8')
#     frame=frame.astype('uint8')
#     if frame.ndim==3: frame=cv.cvtColor(frame,cv.COLOR_BGR2GRAY)
#     if frame.ndim==3: msk=cv.cvtColor(msk,cv.COLOR_BGR2GRAY)
#     mskgrayimg=cv.bitwise_and(frame,frame,mask=msk)
#     cv.imwrite('maskedframecanny.png',mskgrayimg) 
#     if msk is not None:
#         kernel=np.ones((2,2))
#         #dmsk_edge=cv.erode(mskimg,kernel,1)
#         dmsk_edge=cv.dilate(mskimg,kernel,1)
#         cv.imwrite('maskedframecannydilat.png',dmsk_edge) 
#         #_, contours, _ = cv.findContours(dmsk_edge, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
#         _, contours, _ = cv.findContours(dmsk_edge, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
#         if len(contours)>0:
#             print('Length of contour',len(contours))
#             mradius=0
#             mindx=[]
#             mcircent=()
#             for i, cont in enumerate(contours):
#                 (x,y),radius = cv.minEnclosingCircle(cont)
#                 #mcircent=(int(x),int(y))
#                 if mradius<radius:
#                    mindx=i
#                    mcircent=(int(x),int(y))
#                    mradius=int(radius)
#             print('Radius,centre',mradius,mcircent)
#             #kernel=np.ones((2,2))
#             #dmsk_edge=cv.erode(mskimg,kernel,1)
#             #dmsk_edge=cv.dilate(dmsk_edge,kernel,1)
#             if mcircent[1]-mradius>0:    
#                mskgrayimg[:(mcircent[1]-mradius),:]=0;
#             if mcircent[1]+mradius<mskimg.shape[0]:
#                 mskgrayimg[(mcircent[1]+mradius):,:]=0;
#             if mcircent[0]-mradius>0:
#                 mskgrayimg[:,:(mcircent[0]-mradius)]=0;
#             if mcircent[0]+mradius<mskimg.shape[1]:
#                 mskgrayimg[:,(mcircent[0]+mradius):]=0;
#             cv.imwrite('maskedframecannydila_inphough.png',mskgrayimg)
  
#         #circles=cv.HoughCircles(mskimg, cv.HOUGH_GRADIENT,1,20,param1=20,param2=9,minRadius=100,maxRadius=220)
#         #circles=cv.HoughCircles(cv.cvtColor(mskimg,cv.COLOR_BGR2GRAY),cv.HOUGH_GRADIENT,1,20,param1=100,param2=30,minRadius=100,maxRadius=220)
    

#         #circles=cv.HoughCircles(cv.cvtColor(mskimg,cv.COLOR_BGR2GRAY),cv.HOUGH_GRADIENT,1,20,cannythreshold1,cannythreshold2,minRadius,maxRadius)
#         #circles=cv.HoughCircles(mskimg,cv.HOUGH_GRADIENT,1,20,cannythreshold1,cannythreshold2,minRadius,maxRadius)
#         #circles=cv.HoughCircles(mskimg,cv.HOUGH_GRADIENT,1,10)
#         #circles=cv.HoughCircles(mskgrayimg,cv.HOUGH_GRADIENT,2,20,cannythreshold1,cannythreshold2,minRadius,maxRadius)
#     #print(cannythresh,minmaxrang)
    
#     circles=cv.HoughCircles(mskgrayimg,cv.HOUGH_GRADIENT,add_par[0],add_par[1],None,cannythresh[0],cannythresh[1],minmaxrang[0],minmaxrang[1])
#     #circles=cv.HoughCircles(mskimg,cv.HOUGH_GRADIENT_ALT,1,10)
#     if circles is not None:
#         #print('Matrix shape with circle data:',circles.shape,circles)
#         circles = np.uint16(np.around(circles))
#         dist=frame.shape[0]#initialize to a larger number
#         #clos_cent=np.array([frame.shape[1],frame.shape[0]])#initialize to a high value
#         clos_cent=np.array([])
#         for i in circles[0,:]:
#             if i.size != 1:#remove default results
#                 #if stg!=5:#Hole search stage
#                 #    cond=(np.linalg.norm(np.array([i[0],i[1]])-clos_cent[:2])<dist)
#                 #else:
#                 #    cond=(i[2]<minmaxrang[1] and i[2]>minmaxrang[0])  
#                 if np.linalg.norm(np.array([i[0],i[1]])-cent_rad_mask[:2])<dist:#find center closest to the circle detected using mask
#                     clos_cent=np.array([i[0],i[1],i[2]])
#                     #########################
#                     #RAB on 23/2/21
#                     #######################
#                     dist=np.linalg.norm(clos_cent[:2]-cent_rad_mask[:2])
#                     ##############################
#                 # draw the outer circle
#                 cv.circle(frame,(i[0],i[1]),i[2],(0,255,0),2)
#                 # draw the center of the circle
#                 cv.circle(frame,(i[0],i[1]),2,(0,0,255),3)
#             else:  print('No detection')
#         if clos_cent.size!=0:        
#           cv.circle(frame,(clos_cent[0],clos_cent[1]),clos_cent[2],(255,255,255),6)
#           # draw the center of the circle
#           cv.circle(frame,(clos_cent[0],clos_cent[1]),4,(255,255,0),3)
#           #####################
#           #Riby edit 24/2/21
#           ####################
#           #imgcoordhole=np.array([clos_cent[1],clos_cent[0]])
#           imgcoordhole=np.array([clos_cent[0],clos_cent[1]])
#           ####################
#           #print('Coordinate input',imgcoordhole,zcord)
#           print('Coordinate input',imgcoordhole,zcord)
#           worldcoord,imagePoints_proj=holeposdetect(imgcoordhole,zcord)
#           posemat,CE,CCM,com_pose=readpos(posefile_read)
#           com_pose.pos[0]=worldcoord[0]
#           com_pose.pos[1]=worldcoord[1]
#           com_pose.pos[2]=zcord
#         else:
#           com_pose=None
#     else:  
#     	com_pose=None
# 	with open( 'bo7sen' + '.txt', 'w') as file:
# 		file.write(str(X[0]) + " " + str(Y[0]) + " " + str(Z[0]))
# 	return([Xc,Yc,Zc])

    
"""
        Trying out cv2.HoughCircles to detect the circle face of the cylinder 
"""
def detectCircles(image):
	global XY
	output = image.copy()
	#     gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
	gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
	#     G_gray = cv2.GaussianBlur(gray,(11,11),0)
	#     M_gray = cv2.medianBlur(G_gray,9)
	#     showInRow([binary])  
	circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1.2, 500, param1=200, param2=50, maxRadius=50)     # 1.5, 100, param1=200, param2=50
	#     print(circles.shape)
	# ensure at least some circles were found
	if circles is not None:
		# convert the (x, y) coordinates and radius of the circles to integers
		circles = np.round(circles[0, :]).astype("int")
		# loop over the (x, y) coordinates and radius of the circles
		for (x, y, r) in circles:
			# draw the circle in the output image, then draw a rectangle
			# corresponding to the center of the circle
			# z = 220     # z distance from image plane
			z = (41 * f) / (2*r)
			[X,Y,Z] = np.dot(Inv_intrinsic_Mtx, z*np.array([[x],[y],[1]]))
			cv2.circle(output, (x, y), r, (0, 255, 0), 4)
			cv2.rectangle(output, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)
			cv2.putText(output,"{:.2f},{:.2f},{:.2f}".format(X[0],Y[0],Z[0]), (x+r,y) , cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)       # show the output image
			# showInRow([cyl1, output])
			#       showInRow([output])
			#       cv2.waitKey(0)
			# XY.append(sqrt(X[0]**2 + Y[0]**2))
			# if int(XY[-1]) <= int(min(XY)):
			with open( 'bo7sen' + '.txt', 'w') as file:
				file.write(str(X[0]) + " " + str(Y[0]) + " " + str(Z[0]))
				# 			# XY = []
			# else:
			# 	pass
	return output

"""
        Trying out cv2.HoughCircles to detect the circle face of the cylinder ☼
"""
XY = []
intrinsic_Mtx = np.load("mtx_logiRobot.npy")                 # Intrinsics Matrix of the Webcam of My Laptop obtained from Calibration
Inv_intrinsic_Mtx = np.linalg.inv(intrinsic_Mtx)   # Computing the Inverse Matrix of the Intrinsics Matrix
f = sum([intrinsic_Mtx[0,0],intrinsic_Mtx[1,1]])/2
process_video(2, detectCircles)