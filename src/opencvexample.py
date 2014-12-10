#!/usr/bin/env python
import cv2
import urllib
import numpy as np
import math

class Paper:
    bottomleftcorner = tuple([0,0])
    toprightcorner = tuple([0,0])
    def is_within(self, point):
        if point[0] > self.bottomleftcorner[0] and point[0] < self.toprightcorner[0] and point[1] < self.bottomleftcorner[1] and point[1] > self.toprightcorner[1]:
	    return True
	else:
	    return False
	    
print "loading from file"
image = cv2.imread('/home/ubuntu/catkin_ws/src/robot_eksamensprojekt/src/hus.jpg')
#cv2.imshow('raw image',image)

hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

cv2.imshow('hsv',hsv)

lower_red = np.array([50,10,70])
upper_red = np.array([151,110,180])

lower_white = np.array([150,150,150])
upper_white = np.array([255,255,255])

mask_red = cv2.inRange(image, lower_red, upper_red)
mask_white = cv2.inRange(image, lower_white, upper_white)

res_red = cv2.bitwise_and(image,image, mask= mask_red)
res_white = cv2.bitwise_and(image,image, mask= mask_white)

edges_white = cv2.Canny(res_white,150,250)

contours_white,hierarchy = cv2.findContours(edges_white,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)

bricks = []
for cnt in contours_white:
  
    epsilon = 0.1*cv2.arcLength(cnt,True)
    approx = cv2.approxPolyDP(cnt,epsilon,True)
    rect = cv2.minAreaRect(approx)
    area = cv2.contourArea(cnt)   
    #box = cv2.cv.BoxPoints(rect)
    #box = box = np.int0(box)
    if area > 40000:                
        cv2.drawContours(res_white,cnt,-1,(0,255,0),3)      
        leftmost = tuple(cnt[cnt[:,:,0].argmin()][0])
        rightmost = tuple(cnt[cnt[:,:,0].argmax()][0])
        topmost = tuple(cnt[cnt[:,:,1].argmin()][0])
        bottommost = tuple(cnt[cnt[:,:,1].argmax()][0])
        bottomLeftCorner = tuple([leftmost[0], bottommost[1]])
        topRightCorner = tuple([rightmost[0], topmost[1]])
        print bottomLeftCorner
        print topRightCorner
        
leftpaper = Paper()
leftpaper.bottomleftcorner = tuple([24, 268])
leftpaper.toprightcorner = tuple ([291, 80])

print leftpaper.is_within(tuple([96, 236]))

cv2.imshow('raw',image)
#cv2.imshow('mask_red', mask_red)
cv2.imshow('mask_white', mask_white)
#cv2.imshow('res_red',res_red)
cv2.imshow('res_white',res_white)
#cv2.imwrite('result.jpg',image)
cv2.imshow('edges_white',edges_white)

key = cv2.waitKey(0)
if key == 27:
    exit(0)
