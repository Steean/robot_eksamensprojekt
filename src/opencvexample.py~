#!/usr/bin/env python
import cv2
import urllib
import numpy as np
import math

def get_from_webcam():
    """
    Fetches an image from the webcam
    """
    stream=urllib.urlopen('http://192.168.0.20/image/jpeg.cgi')
    bytes=''
    bytes+=stream.read(64500)
    a = bytes.find('\xff\xd8')
    b = bytes.find('\xff\xd9')

    if a != -1 and b != -1:
        jpg = bytes[a:b+2]
        bytes= bytes[b+2:]
        i = cv2.imdecode(np.fromstring(jpg, dtype=np.uint8),cv2.CV_LOAD_IMAGE_COLOR)
        return i
    else:
        print "did not receive image, try increasing the buffer size in line 13:"

def get_from_file(filename):
    """
    Loads image from file
    """
    return cv2.imread(filename)

def get_bricks(contours):
    """
    For each contour in contours
        approximate the contours such that small variations are removed
        calulate the area of the contour
        if the area is within the desired range we append the box points to the
        bricks.
    """
    bricks = []
    for cnt in contours:
        epsilon = 0.1*cv2.arcLength(cnt,True)
        approx = cv2.approxPolyDP(cnt,epsilon,True)
        rect = cv2.minAreaRect(approx)
        area = cv2.contourArea(approx)
        box = cv2.cv.BoxPoints(rect)
        box = box = np.int0(box)
        if area > 500 and area < 2400:
            bricks.append(box)
    return bricks


def extract_single_color_range(image,hsv,lower,upper):
    """
    Calculates a mask for which all pixels within the specified range is set to 1
    the ands this mask with the provided image such that color information is
    still present, but only for the specified range
    """
    mask = cv2.inRange(hsv, lower, upper)
    res = cv2.bitwise_and(image,image, mask= mask)
    return res

def threshold_image(image,debug=False):
    """
    Thresholds the image within the desired range and then dilates with a 3x3 matrix
    such that small holes are filled. Afterwards the 'blobs' are closed using a
    combination of dilate and erode
    """
    ret,th1 = cv2.threshold(image,50,255,cv2.THRESH_BINARY)
    if debug: cv2.imshow('th1',th1)
    resdi = cv2.dilate(th1,np.ones((3,3),np.uint8))
    if debug: cv2.imshow('dilated',resdi)
    closing = cv2.morphologyEx(resdi, cv2.MORPH_CLOSE,np.ones((5,5),np.uint8))
    if debug: cv2.imshow('closing',closing)

    return closing

def contours(image,debug=False):
    """
    Extract the contours of the image by first converting it to grayscale and then
    call findContours
    """
    imgray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
    if debug: cv2.imshow('gray_scale_contour',imgray)
    contours, hierarchy = cv2.findContours(imgray,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

    return contours,hierarchy


def do_full(image,hsv,upper,lower,debug=False):
    """
    Main methods for processing an image and detect rectangles in the given
    hsv color range

    set debug to True in order to show the intermediate images
    """
    single_color_img = extract_single_color_range(image,hsv,lower,upper)
    if debug:
        cv2.imshow('single_color_img',single_color_img)
    single_channel = threshold_image(single_color_img,debug)
    if debug:
        cv2.imshow('single_channel',single_channel)
    cont,hierarchy = contours(single_channel,debug)

    if debug:
        for i,cnt in enumerate(cont):
            cv2.drawContours(single_channel,cont,i,(0,0,255),2)
    if debug: cv2.imshow('contours',single_channel)

    return get_bricks(cont)

def show_bricks(image,bricks,color):
    for b in bricks:
        cv2.drawContours(image,[b],0,color,2)

print "loading from file"
image = get_from_file('hus.jpg')
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

cv2.imshow('raw',image)
cv2.imshow('mask_red', mask_red)
cv2.imshow('mask_white', mask_white)
cv2.imshow('res_red',res_red)
cv2.imshow('res_white',res_white)
#cv2.imwrite('result.jpg',image)

key = cv2.waitKey(0)
if key == 27:
    exit(0)
