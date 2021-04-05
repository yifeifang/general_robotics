#! /usr/bin/python
#

import cv2
import numpy as np
import math

img = cv2.imread('frame_ROS_camera.jpeg')

# Change image to grayscale
gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY) 

# Use Canny edge detector to detect edges on the image
edges = cv2.Canny(gray,100,200,apertureSize = 3)

# Use Hough Line Transform to detect straight lines. 
# Line parameters are stored in lines tuple
lines = cv2.HoughLines(edges, 1, np.pi / 180, 100, None, 0, 0)

if lines is not None:
    for i in range(0, len(lines)):
        rho = lines[i][0][0]
        theta = lines[i][0][1]
        a = math.cos(theta)
        b = math.sin(theta)
        x0 = a * rho
        y0 = b * rho
        pt1 = (int(x0 + 1000*(-b)), int(y0 + 1000*(a)))
        pt2 = (int(x0 - 1000*(-b)), int(y0 - 1000*(a)))
        cv2.line(img, pt1, pt2, (0,0,255), 3, cv2.LINE_AA)

# Displaying the image 
cv2.imshow("Result",img)

#waits for user to press any key 
cv2.waitKey(0) 

#closing all open windows 
cv2.destroyAllWindows()