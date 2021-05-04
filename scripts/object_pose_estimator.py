#! /usr/bin/python

# rospy for the subscriber
import rospy
# ROS messages
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import Point
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for processing and saving an image
import cv2
# Numpy to work with arrays
import numpy as np
import struct
# helper function for TF transformations
from tf_listener import get_transformation, transform_point


# Instantiate CvBridge
bridge = CvBridge()

def segment_image(img):
    lower_blue = np.array([100,0,0])
    upper_blue = np.array([150,10,10])
    mask = cv2.inRange(img, lower_blue, upper_blue)

    obj_seg = cv2.bitwise_and(img,img, mask=mask)

    # Find the moments of the image
    M = cv2.moments(mask)
    print M

    if M['m00'] != 0:
        cX = int(M['m10'] / M['m00'])
        cY = int(M['m01'] / M['m00'])
    else:
        cX, cY = 0, 0

    cv2.circle(obj_seg, (cX, cY), 5, (0, 0, 255), -1)

    cv2.imshow("Result",obj_seg)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    return cX, cY, M['m00']

def get_XYZ(msg_pcl,cX,cY):
    array_position = cY*msg_pcl.row_step + cX*msg_pcl.point_step
    camPoint = Point()
    (camPoint.x, camPoint.y, camPoint.z) = struct.unpack_from('fff', msg_pcl.data, offset=array_position)

    return camPoint

def main():
    rospy.init_node('pose_estimator')
    # Define your image topics
    color_topic = "/head_camera/rgb/image_raw"
    pcl_topic = "/head_camera/depth_registered/points"
    # Set up your color subscriber and define its callback
    #rospy.Subscriber(color_topic, Image, color_image_callback)
    # Set up your depth subscriber and define its callback
    #rospy.Subscriber(pcl_topic, PointCloud2, ptcloud_callback)
    msg = rospy.wait_for_message(color_topic,Image)
    print("Received a color image!")
    try:
        # Convert your ROS Image message to OpenCV2
        cv2_color_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError, e:
        print(e)
    else:
        # Save your OpenCV2 image as a jpeg 
        cv2.imshow("Image",cv2_color_img)
        cv2.waitKey(0) 

        #closing all open windows 
        cv2.destroyAllWindows()

    cX,cY,area = segment_image(cv2_color_img)

    print "Centroid, x: ", cX, "y: ", cY
    print "Area: ", area

    msg_pcl = rospy.wait_for_message(pcl_topic,PointCloud2)
    print("Received pointcloud")

    camP = get_XYZ(msg_pcl,cX,cY)

    print camP

    # define source and target frame
    source_frame = 'head_camera_rgb_optical_frame'
    target_frame = 'base_link'

    transformation = get_transformation(source_frame, target_frame)
    point_wrt_target = transform_point(transformation, camP)

    print point_wrt_target

    #Options: 
    # 1) publish point_wrt_target in a Publisher for another node to move the arm
    # 2) import fetch_api and use arm class to provide Pose stamped as goal to the gripper
    # 3) draw from helper function from first course to move fetch to a given pose

    
    # Spin until ctrl + c. You can also choose to spin once
    rospy.sleep(0.5)
    rospy.signal_shutdown('Done!')

if __name__ == '__main__':
    main()