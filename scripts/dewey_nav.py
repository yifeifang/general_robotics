#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
import tf
import numpy as np
import math

current_pose = None
target_pose = None

def update_current_pose(data):
    global current_pose
    current_pose = data.pose.pose

def update_target_pose(data):
    global target_pose 
    target_pose = data.pose

rospy.init_node('dewey_nav', anonymous=True)
publisher = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, update_current_pose)
rospy.Subscriber("move_base_simple/goal", PoseStamped, update_target_pose)
r = rospy.Rate(25)
while not rospy.is_shutdown():
    if(current_pose and target_pose):
        # Assuming counter clockwise rotation
        # if zeta2 (calculated with arcsin) is negative
        # Than the rotation is counter clockwise
        # if zeta2 is positive
        # Than the rotation is clockwise
        vector_a = np.array([target_pose.position.x - current_pose.position.x, target_pose.position.y - current_pose.position.y])
        vector_b = np.array([current_pose.position.x, current_pose.position.y])

        vector_a_hat = vector_a / np.linalg.norm(vector_a)
        vector_b_hat = vector_b / np.linalg.norm(vector_b)
        zeta1 = math.acos(np.dot(vector_a_hat, vector_b_hat))
        zeta2 = math.asin((vector_a_hat[0] - math.cos(zeta1) * vector_b_hat[0]) / -vector_b_hat[1])
        print zeta1, zeta2
        propotion = 0.3 * np.sign(zeta2) * zeta1

        #print "propotion set to ", propotion
        # twist = Twist()
        # twist.linear.x = 0
        # twist.linear.y = 0
        # twist.linear.z = 0
        # twist.angular.x = 0
        # twist.angular.y = 0
        # twist.angular.z = propotion
        # publisher.publish(twist)
    r.sleep()