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
import fetch_api

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_name = "arm_with_torso"
move_group = moveit_commander.MoveGroupCommander(group_name)
gripper = fetch_api.Gripper()

pose_goal = geometry_msgs.msg.Pose()
target_q = tf.transformations.quaternion_from_euler(0.0, 3.14 / 2.0, 0.0)

constraint = moveit_msgs.msg.Constraints()
constraint.name = "dewey grasp constraint"
orientation_constraint = moveit_msgs.msg.OrientationConstraint()
orientation_constraint.header.frame_id = "base_link"
orientation_constraint.link_name = "wrist_roll_link"
orientation_constraint.orientation = geometry_msgs.msg.Quaternion(target_q[0],target_q[1],target_q[2],target_q[3])
orientation_constraint.absolute_x_axis_tolerance = 0.1
orientation_constraint.absolute_y_axis_tolerance = 0.1
orientation_constraint.absolute_z_axis_tolerance = 0.1
orientation_constraint.weight = 1
constraint.orientation_constraints.append(orientation_constraint)
move_group.set_path_constraints(constraint)

print "target Quaternion = ", target_q[0], target_q[1], target_q[2], target_q[3]
pose_goal.orientation.x = target_q[0]
pose_goal.orientation.y = target_q[1]
pose_goal.orientation.z = target_q[2]
pose_goal.orientation.w = target_q[3]

# Sequence start
gripper.close()

pose_goal.position.x = 0.3
pose_goal.position.y = 0
pose_goal.position.z = 0.7

move_group.set_pose_target(pose_goal)

move_group.set_start_state(move_group.get_current_state())

# move_group.set_planning_time(15)
myplan = move_group.plan()

move_group.execute(myplan)

move_group.set_start_state(move_group.get_current_state())

pose_goal.position.x = 0.8
pose_goal.position.y = 0
pose_goal.position.z = 0.7

move_group.set_pose_target(pose_goal)

myplan = move_group.plan()

move_group.execute(myplan)

gripper.open()

# Calling `stop()` ensures that there is no residual movement
move_group.stop()
# It is always good to clear your targets after planning with poses.
# Note: there is no equivalent function for clear_joint_value_targets()
move_group.clear_pose_targets()