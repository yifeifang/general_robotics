import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose
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
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_name = "arm_with_torso"
move_group = moveit_commander.MoveGroupCommander(group_name)
gripper = Gripper()

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
pose_transformed.pose.position.z += 0.5
print pose_transformed.pose.position.x
print pose_transformed.pose.position.y
print pose_transformed.pose.position.z

angles = euler_from_quaternion([pose_transformed.pose.orientation.x, pose_transformed.pose.orientation.y, pose_transformed.pose.orientation.z, pose_transformed.pose.orientation.w])
target_q = tf.transformations.quaternion_from_euler(0.0, 3.14 / 2.0, angles[2])

while True:
    obj_marker1 = rospy.wait_for_message('box_marker', Marker, timeout=15)
    obj_marker2 = rospy.wait_for_message('box_marker', Marker, timeout=15)
    # if the box maker has scale in y larger than scale in x then we need to turn the gripper 90 degrees
    if obj_marker1.scale.y > obj_marker1.scale.x and obj_marker2.scale.y > obj_marker2.scale.x:
        target_q = tf.transformations.quaternion_multiply(target_q, tf.transformations.quaternion_from_euler(3.14 / 2.0, 0.0, 0.0))
        break
    elif obj_marker1.scale.y < obj_marker1.scale.x and obj_marker2.scale.y < obj_marker2.scale.x:
        break
    else:
        print "inconsistant marker"
        pass

pose_transformed.pose.orientation.x = target_q[0]
pose_transformed.pose.orientation.y = target_q[1]
pose_transformed.pose.orientation.z = target_q[2]
pose_transformed.pose.orientation.w = target_q[3]

move_group.set_pose_target(pose_transformed)

move_group.set_planning_time(15)
myplan = move_group.plan()

# myc = move_group.get_path_constraints()

move_group.execute(myplan)
# Calling `stop()` ensures that there is no residual movement
move_group.stop()
# It is always good to clear your targets after planning with poses.
# Note: there is no equivalent function for clear_joint_value_targets()
move_group.clear_pose_targets()

#################################### approaching
rospy.sleep(3)
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

#################################### retracting
rospy.sleep(3)
gripper.close()
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
pose.position.z += 0.1
waypoints.append(copy.deepcopy(pose))

pose.position.z += 0.15
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

########################################### tuck
rospy.sleep(3)
target_q = tf.transformations.quaternion_from_euler(0.0, 3.14 / 2.0, 0.0)
joint_goal = [0 for i in range(8)]
joint_goal[0] = 0.0857168138027
joint_goal[1] = -1.58481834789
joint_goal[2] = 0.433445118112
joint_goal[3] = -1.53769913621
joint_goal[4] = 1.80451970107
joint_goal[5] = 2.0000173681
joint_goal[6] = 1.63692856729
joint_goal[7] = 2.86215073034

move_group.set_joint_value_target(joint_goal)

constraint = moveit_msgs.msg.Constraints()
constraint.name = "dewey grasp constraint"
orientation_constraint = moveit_msgs.msg.OrientationConstraint()
orientation_constraint.header.frame_id = "base_link"
orientation_constraint.link_name = "wrist_roll_link"
orientation_constraint.orientation = geometry_msgs.msg.Quaternion(target_q[0],target_q[1],target_q[2],target_q[3])
# It looks like it didn't took value < 0.1 into account need to investigate
# in to ompl source for more info
orientation_constraint.absolute_x_axis_tolerance = 2 * pi
orientation_constraint.absolute_y_axis_tolerance = 0.1
orientation_constraint.absolute_z_axis_tolerance = 0.1
orientation_constraint.weight = 1
constraint.orientation_constraints.append(orientation_constraint)
move_group.set_path_constraints(constraint)

# move_group.set_start_state(move_group.get_current_state())

move_group.set_planning_time(15)
myplan = move_group.plan()

move_group.execute(myplan)
# Calling `stop()` ensures that there is no residual movement
move_group.stop()