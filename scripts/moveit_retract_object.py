import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import tf
from tf.transformations import *
import tf2_ros
import tf2_geometry_msgs
import numpy as np
import time
from fetch_api import Gripper

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

gripper = Gripper()
gripper.close()

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_name = "arm_with_torso"
move_group = moveit_commander.MoveGroupCommander(group_name)

waypoints = []
wpose = move_group.get_current_pose().pose
wpose.position.z += 0.1
waypoints.append(copy.deepcopy(wpose))

wpose = move_group.get_current_pose().pose
wpose.position.z += 0.25
waypoints.append(copy.deepcopy(wpose))

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