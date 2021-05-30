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

# from tf_listener import get_transformation, transform_point
# OpenCV2 for saving an image

import cv2

import numpy as np

import math

import actionlib

import fetch_api

import control_msgs.msg

from sensor_msgs.msg import PointCloud2

import trajectory_msgs.msg

import struct

from geometry_msgs.msg import Point, PointStamped, PoseStamped

from robotics_labs.msg import BoxTarget

import tf2_ros
import tf2_geometry_msgs

MIN_PAN = -math.pi / 2
MAX_PAN = math.pi / 2
MIN_TILT = -math.pi / 2
MAX_TILT = math.pi / 4
PAN_JOINT = 'head_pan_joint'
TILT_JOINT = 'head_tilt_joint'
PAN_TILT_TIME = 2

# Instantiate CvBridge
bridge = CvBridge()
initBB = None
tracker = cv2.TrackerCSRT_create()
fetch_base = fetch_api.Base()
linear_speed = 0.3
angular_speed = 0
porpotion_prev = 0  # assuming fetch is facing the object
porpotion_tilt_prev = 0
intergral_tilt = 0
traj_client = None
current_dist = 0
cx = None
cy = None
image_sub = None
dist_sub = None
tilt = 0
tf_buffer = None
stop = False

def dist_callback(msg):
    global current_dist
    global cx
    global cy
    global tilt
    global dist_sub
    global stop

    transform = tf_buffer.lookup_transform("base_link",
                                            "head_camera_rgb_optical_frame", #source frame
                                            rospy.Time(0),
                                            rospy.Duration(5.0)) #get the tf at first available time
    pose_goal = PoseStamped()
    pose_goal.header.frame_id = "head_camera_rgb_optical_frame"
    pose_goal.pose = msg.box_pose
    pose_transformed = tf2_geometry_msgs.do_transform_pose(pose_goal, transform)

    current_dist = pose_transformed.pose.position.x

    if (not math.isnan(current_dist)) and current_dist < 0.82 and current_dist != 0:
        stop = True

    print "current_dist = ", current_dist

def image_callback(msg):
    global initBB
    global tracker
    global fetch_base
    global linear_speed
    global angular_speed
    global porpotion_prev
    global porpotion_tilt
    global traj_client
    global intergral_tilt
    global cx
    global cy
    global image_sub
    global tilt
    # print("Received an image!")
    # if (not math.isnan(current_dist)) and current_dist < 0.85 and current_dist != 0:
    #     fetch_base.move(0, 0)
    #     image_sub.unregister()
    try:
        # Convert your ROS Image message to OpenCV2
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError, e:
        print(e)
    else:
        # Save your OpenCV2 image as a jpeg 
        # pass
        # Change image to grayscale
        height = cv2_img.shape[0]
        width = cv2_img.shape[1]
        if not initBB:
            initBB = cv2.selectROI("Frame", cv2_img, fromCenter=False, showCrosshair=True)
            tracker.init(cv2_img, initBB)
        else:
            (success, box) = tracker.update(cv2_img)
            (x, y, w, h) = [int(v) for v in box]
            cv2.rectangle(cv2_img, (x, y), (x + w, y + h), (0, 255, 0), 2)

            cx = x + w / (2 * 1.0)
            cy = y + h / (2 * 1.0)

            precentage = cx / (width * 1.0) 
            precentage_height = cy / (height * 1.0) 
            
            porpotion = -(precentage - 0.5)
            differential = porpotion - porpotion_prev

            porpotion_tilt = (precentage_height - 0.5)
            differential_tilt = porpotion_tilt - porpotion_tilt
            intergral_tilt += (precentage_height - 0.5)

            angular_speed = porpotion + 0.001 * differential
            tilt = 5 * porpotion_tilt + 0.01 * differential_tilt + 0.001 * intergral_tilt
            # print porpotion_tilt, differential_tilt, tilt
            porpotion_prev = porpotion
            porpotion_tilt_prev = porpotion_tilt
            # Displaying the image 
            fetch_base.move(linear_speed, angular_speed)

            point = trajectory_msgs.msg.JointTrajectoryPoint()
            point.positions = [0, tilt]
            point.time_from_start = rospy.Duration(PAN_TILT_TIME)
            goal = control_msgs.msg.FollowJointTrajectoryGoal()

            goal.trajectory.joint_names = [PAN_JOINT, TILT_JOINT]
            goal.trajectory.points.append(point)
            traj_client.send_goal(goal)
            cv2.imshow("Result",cv2_img)
            cv2.waitKey(1)



def main():
    global traj_client
    global image_sub
    global dist_sub
    global tf_buffer
    rospy.init_node('dewey_fetch_nav')
    tf_buffer = tf2_ros.Buffer(rospy.Duration(1))
    tf2_ros.TransformListener(tf_buffer)
    traj_client = actionlib.SimpleActionClient('head_controller/follow_joint_trajectory', control_msgs.msg.FollowJointTrajectoryAction)
    # Define your image topic
    image_topic = "/head_camera/rgb/image_raw"
    # Set up your subscriber and define its callback
    image_sub = rospy.Subscriber(image_topic, Image, image_callback)
    dist_sub = rospy.Subscriber("box_target", BoxTarget, dist_callback)
    # Spin until ctrl + c. You can also choose to spin once
    while not rospy.is_shutdown():
        if stop:
            image_sub.unregister()
            dist_sub.unregister()
        rospy.sleep(0.1)
    rospy.spin()

if __name__ == '__main__':
    main()