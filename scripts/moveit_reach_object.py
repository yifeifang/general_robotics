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

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_name = "arm_with_torso"
move_group = moveit_commander.MoveGroupCommander(group_name)

tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0)) #tf buffer length
tf_listener = tf2_ros.TransformListener(tf_buffer)

transform = tf_buffer.lookup_transform("base_link",
                                       "head_camera_rgb_optical_frame", #source frame
                                       rospy.Time(0),
                                       rospy.Duration(5.0)) #get the tf at first available time

obj_pose = rospy.wait_for_message('box_target_pose', geometry_msgs.msg.Pose, timeout=15)
pose_goal = geometry_msgs.msg.PoseStamped()
pose_goal.header.frame_id = "head_camera_rgb_optical_frame"
pose_goal.pose = obj_pose
pose_transformed = tf2_geometry_msgs.do_transform_pose(pose_goal, transform)

# listener = TransformListener()
# target_pose = listener.transformPose("base_link", pose_transformed)
pose_transformed.pose.position.z += 0.4
print pose_transformed.pose.position.x
print pose_transformed.pose.position.y
print pose_transformed.pose.position.z

angles = euler_from_quaternion([pose_transformed.pose.orientation.x, pose_transformed.pose.orientation.y, pose_transformed.pose.orientation.z, pose_transformed.pose.orientation.w])
target_q = tf.transformations.quaternion_from_euler(0.0, 3.14 / 2.0, angles[2])

pose_transformed.pose.orientation.x = target_q[0]
pose_transformed.pose.orientation.y = target_q[1]
pose_transformed.pose.orientation.z = target_q[2]
pose_transformed.pose.orientation.w = target_q[3]

move_group.set_pose_target(pose_transformed)

# Maybe the robot planner didn't take it into account? Need to check move_group.launch
# Check https://github.com/ros-planning/moveit/pull/541 for detail problem due to 
# fetch_moveit_config/config/ompl_planner.yaml is modified
# Check if sampling in JointModelStateSpace is enforced for this group by user.
# This is done by setting 'enforce_joint_model_state_space' to 'true' for the desired group in ompl_planning.yaml.

# Some planning problems like orientation path constraints are represented in PoseModelStateSpace and sampled via IK.
# However consecutive IK solutions are not checked for proximity at the moment and sometimes happen to be flipped,
# leading to invalid trajectories. This workaround lets the user prevent this problem by forcing rejection sampling
# in JointModelStateSpace.

# constraint = moveit_msgs.msg.Constraints()
# constraint.name = "dewey grasp constraint"
# orientation_constraint = moveit_msgs.msg.OrientationConstraint()
# orientation_constraint.header.frame_id = "base_link"
# orientation_constraint.link_name = "wrist_roll_link"
# orientation_constraint.orientation = geometry_msgs.msg.Quaternion(pose_transformed.pose.orientation.x,
#                                                                   pose_transformed.pose.orientation.y,
#                                                                   pose_transformed.pose.orientation.z,
#                                                                   pose_transformed.pose.orientation.w)
# # It looks like it didn't took value < 0.1 into account need to investigate
# # in to ompl source for more info
# orientation_constraint.absolute_x_axis_tolerance = 0.1
# orientation_constraint.absolute_y_axis_tolerance = 0.1
# orientation_constraint.absolute_z_axis_tolerance = 0.1
# orientation_constraint.weight = 1
# constraint.orientation_constraints.append(orientation_constraint)
# move_group.set_path_constraints(constraint)

# move_group.set_start_state(move_group.get_current_state())

move_group.set_planning_time(15)
myplan = move_group.plan()

# myc = move_group.get_path_constraints()

move_group.execute(myplan)
# Calling `stop()` ensures that there is no residual movement
move_group.stop()
# It is always good to clear your targets after planning with poses.
# Note: there is no equivalent function for clear_joint_value_targets()
move_group.clear_pose_targets()

# time.sleep(3)

# waypoints = []
# wpose = move_group.get_current_pose().pose
# wpose.position.z -= 0.1
# waypoints.append(copy.deepcopy(wpose))

# wpose = move_group.get_current_pose().pose
# wpose.position.z -= 0.2
# waypoints.append(copy.deepcopy(wpose))

# (plan, fraction) = move_group.compute_cartesian_path(
#                                    waypoints,   # waypoints to follow
#                                    0.01,        # eef_step
#                                    0.0,
#                                    False)         # jump_threshold

# move_group.execute(plan)
# # Calling `stop()` ensures that there is no residual movement
# move_group.stop()
# # It is always good to clear your targets after planning with poses.
# # Note: there is no equivalent function for clear_joint_value_targets()
# move_group.clear_pose_targets()