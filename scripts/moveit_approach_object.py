import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from moveit_commander.conversions import pose_to_list
import tf
from tf.transformations import *
import tf2_ros
import tf2_geometry_msgs
import numpy as np
import time

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_name = "arm_with_torso"
move_group = moveit_commander.MoveGroupCommander(group_name)

tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0)) #tf buffer length
tf_listener = tf2_ros.TransformListener(tf_buffer)
transform = tf_buffer.lookup_transform("base_link",
                                       "wrist_roll_link", #source frame
                                       rospy.Time(0),
                                       rospy.Duration(5.0)) #get the tf at first available time

pose = Pose()
pose.position.x = transform.transform.translation.x
pose.position.y = transform.transform.translation.y
pose.position.z = transform.transform.translation.z
pose.orientation.x = transform.transform.rotation.x
pose.orientation.y = transform.transform.rotation.y
pose.orientation.z = transform.transform.rotation.z
pose.orientation.w = transform.transform.rotation.w

waypoints = []
pose.position.z -= 0.1
waypoints.append(copy.deepcopy(pose))

pose.position.z -= 0.08
waypoints.append(copy.deepcopy(pose))

(plan, fraction) = move_group.compute_cartesian_path(
                                   waypoints,   # waypoints to follow
                                   0.01,        # eef_step
                                   0.0,
                                   False)         # jump_threshold

move_group.execute(plan)
# Calling `stop()` ensures that there is no residual movement
move_group.stop()
# It is always good to clear your targets after planning with poses.
# Note: there is no equivalent function for clear_joint_value_targets()
move_group.clear_pose_targets()