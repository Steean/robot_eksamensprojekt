#!/usr/bin/env python
import cv2
import urllib
import numpy as np
import math

class Contour:
    def __init__(self, cnt):
	self.contour = cnt
        leftmost = tuple(cnt[cnt[:,:,0].argmin()][0])
        rightmost = tuple(cnt[cnt[:,:,0].argmax()][0])
        topmost = tuple(cnt[cnt[:,:,1].argmin()][0])
        bottommost = tuple(cnt[cnt[:,:,1].argmax()][0])
        self.bottomLeftCorner = tuple([leftmost[0], bottommost[1]])
        self.topRightCorner = tuple([rightmost[0], topmost[1]])	

    bottomLeftCorner = tuple([0,0])
    topRightCorner = tuple([0,0])
    contour = []
    def is_within(self, point):
        if point[0] > self.bottomLeftCorner[0] and point[0] < self.topRightCorner[0] and point[1] < self.bottomLeftCorner[1] and point[1] > self.topRightCorner[1]:
	    return True
	else:
	    return False

    def is_equal(self, cnt, tolerance = 0):
		if math.fabs(cnt.bottomLeftCorner[0]-self.bottomLeftCorner[0]) < tolerance and math.fabs(cnt.bottomLeftCorner[1]-self.bottomLeftCorner[1])<tolerance:
			return True
		else:
			return False   
	
# Color definitions
lower_red = np.array([50,10,70])
upper_red = np.array([151,110,180])

lower_white = np.array([150,150,150])
upper_white = np.array([255,255,255])
    
# Main
print "loading from file"
image = cv2.imread('/home/ubuntu/catkin_ws/src/robot_eksamensprojekt/src/hus.jpg')
cv2.imshow('raw image',image)

#hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
#cv2.imshow('hsv',hsv)

mask_red = cv2.inRange(image, lower_red, upper_red)
mask_white = cv2.inRange(image, lower_white, upper_white)

res_red = cv2.bitwise_and(image,image, mask= mask_red)
res_white = cv2.bitwise_and(image,image, mask= mask_white)

edges_white = cv2.Canny(res_white,150,250)

contours_white,hierarchy = cv2.findContours(edges_white,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)

papers = []
for cnt in contours_white:   
    if cv2.contourArea(cnt) > 40000:              	
		is_new = True
		cv2.drawContours(res_white,cnt,-1,(0,255,0),1)      
		c = Contour(cnt)		

		for p in papers:
			if c.is_equal(p,10):
				is_new = False

		if is_new:
			papers.append(c)
        
for p in papers:
    print cv2.contourArea(p.contour)

inside = []
for cnt in contours_white:
	c = Contour(cnt)
	if papers[0].is_within(c.bottomLeftCorner):
		is_new = True
		for i in inside:
			if c.is_equal(i,15):
				is_new = False

	if is_new:
		inside.append(c)     

points = []
for i in inside:
    cv2.drawContours(res_white,i.contour,-1,(0,0,255),3)
    points.append(cv2.approxPolyDP(i.contour, 3, False))

for p in points:
	for po in p:
		print po
		cv2.circle(image, (po[0][0], po[0][1]), 1, 0, 1)
	
	print "------"

#cv2.imshow('raw',image)
#cv2.imshow('mask_red', mask_red)
#cv2.imshow('mask_white', mask_white)
#cv2.imshow('res_red',res_red)
cv2.imshow('res_white',res_white)
#cv2.imwrite('result.jpg',image)
#cv2.imshow('edges_white',edges_white)

key = cv2.waitKey(0)
if key == 27:
    exit(0)
