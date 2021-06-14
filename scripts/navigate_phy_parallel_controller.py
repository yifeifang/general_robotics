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
from sensor_msgs.msg import Image, JointState
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

from threading import Thread
from multiprocessing import Process, Value, Array

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
linear_speed = 0
angular_speed = Value('d', 0.0)
porpotion_prev = 0  # assuming fetch is facing the object
e_prev = 0
intergral_tilt = 0
traj_client = None
current_dist = 0
cx = None
cy = None
image_sub = None
dist_sub = None
tilt = Value('d', 0.0)
tf_buffer = None
stop = False
flush_count = 0
current_head_angle = 0

def controller():
    print "process running"
    global angular_speed
    global linear_speed
    global tilt
    global traj_client
    global stop
    fetch_base = fetch_api.Base()
    fetch_head = fetch_api.Head()
    r = rospy.Rate(25)
    while not rospy.is_shutdown() and not stop:
        # print "publishing"
        fetch_base.move(linear_speed, angular_speed.value)
        point = trajectory_msgs.msg.JointTrajectoryPoint()
        point.positions = [0, tilt.value]
        point.time_from_start = rospy.Duration(2.5)
        goal = control_msgs.msg.FollowJointTrajectoryGoal()

        goal.trajectory.joint_names = [PAN_JOINT, TILT_JOINT]
        goal.trajectory.points.append(point)
        traj_client.send_goal(goal)
        
        r.sleep()

    # fetch_base.go_forward(0.35)
    fetch_head.pan_tilt(0,0.8)

    return

def joint_callback(msg):
    try:
        current_head_angle = msg.position[5]
    except:
        pass
    # print current_head_angle

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

    if (not math.isnan(current_dist)) and current_dist < 0.92 and current_dist != 0:
        print "stop set"
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
    global flush_count
    global e_prev
    # print("Received an image!")
    # if (not math.isnan(current_dist)) and current_dist < 0.85 and current_dist != 0:
    #     fetch_base.move(0, 0)
    #     image_sub.unregister()
    if flush_count < 5:
        flush_count += 1
        return
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
            linear_speed = 0.15
            (success, box) = tracker.update(cv2_img)
            (x, y, w, h) = [int(v) for v in box]
            cv2.rectangle(cv2_img, (x, y), (x + w, y + h), (0, 255, 0), 2)

            cx = x + w / (2 * 1.0)
            cy = y + h / (2 * 1.0)

            precentage = cx / (width * 1.0) 
            precentage_height = cy / (height * 1.0) 
            
            porpotion = -(precentage - 0.5)
            differential = porpotion - porpotion_prev

            SV = -0.8 + 2 * precentage_height#(precentage_height - 0.5)
            e = SV
            porpotion_tilt = e
            differential_tilt = e - e_prev
            # intergral_tilt += (precentage_height - 0.5)

            angular_speed.value = porpotion + 0.001 * differential
            tilt.value = current_head_angle + 1.5 * porpotion_tilt + 0.5 * differential_tilt

            if tilt.value < 0.2:
                tilt.value = 0.2

            print "SV = ", SV, ", precentage_height = ", precentage_height
            porpotion_prev = porpotion
            e_prev = e
            # Displaying the image 
            # fetch_base.move(linear_speed, angular_speed)

            # point = trajectory_msgs.msg.JointTrajectoryPoint()
            # point.positions = [0, tilt]
            # point.time_from_start = rospy.Duration(PAN_TILT_TIME)
            # goal = control_msgs.msg.FollowJointTrajectoryporpotion_tiltbporpotion_tiltporpotion_tiltGoal()

            # goal.trajectory.joint_names = [PAN_JOINT, TILT_JOINT]
            # goal.trajectory.points.append(point)
            # traj_client.send_goal(goal)
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
    joint_sub = rospy.Subscriber("joint_states", JointState, joint_callback)
    t = Thread(target=controller)
    t.start()
    # Spin until ctrl + c. You can also choose to spin once
    while not rospy.is_shutdown():
        if stop:
            image_sub.unregister()
            dist_sub.unregister()
            joint_sub.unregister()
        rospy.sleep(0.1)
    rospy.spin()
    t.join()
if __name__ == '__main__':
    main()