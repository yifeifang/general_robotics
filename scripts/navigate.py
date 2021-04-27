#! /usr/bin/python
# Copyright (c) 2015, Rethink Robotics, Inc.

# Using this CvBridge Tutorial for converting
# ROS images to OpenCV2 images
# http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython

# Using this OpenCV2 tutorial for saving Images:
# http://opencv-python-tutroals.readthedocs.org/en/latest/py_tutorials/py_gui/py_image_display/py_image_display.html

# rospy for the subscriber
import rospy
# ROS Image message
from sensor_msgs.msg import Image
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image

import cv2

import numpy as np

import math

import fetch_api

# Instantiate CvBridge
bridge = CvBridge()
initBB = None
tracker = cv2.TrackerCSRT_create()
fetch_base = fetch_api.Base()
linear_speed = 0.1
angular_speed = 0

def image_callback(msg):
    global initBB
    global tracker
    global fetch_base
    global linear_speed
    global angular_speed
    print("Received an image!")
    try:
        # Convert your ROS Image message to OpenCV2
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError, e:
        print(e)
    else:
        # Save your OpenCV2 image as a jpeg 
        # pass
        # Change image to grayscale
        width = cv2_img.shape[1]
        if not initBB:
            initBB = cv2.selectROI("Frame", cv2_img, fromCenter=False, showCrosshair=True)
            tracker.init(cv2_img, initBB)
        else:
            (success, box) = tracker.update(cv2_img)
            (x, y, w, h) = [int(v) for v in box]
            cv2.rectangle(cv2_img, (x, y), (x + w, y + h), (0, 255, 0), 2)

            target_width = x + w / (2 * 1.0)

            precentage = target_width / (width * 1.0) 

            angular_speed = -(precentage - 0.5)
            
            # Displaying the image 
            fetch_base.move(linear_speed, angular_speed)
            cv2.imshow("Result",cv2_img)
            cv2.waitKey(1)



def main():
    rospy.init_node('dewey_fetch_nav')
    # Define your image topic
    image_topic = "/head_camera/rgb/image_raw"
    # Set up your subscriber and define its callback
    rospy.Subscriber(image_topic, Image, image_callback)
    # Spin until ctrl + c. You can also choose to spin once
    rospy.spin()

if __name__ == '__main__':
    main()