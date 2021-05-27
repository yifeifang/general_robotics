import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import numpy as np
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import tf

def wait_for_state_update(box_name, scene, box_is_known=False, box_is_attached=False, timeout=4):
    box_name = box_name
    scene = scene
    start = rospy.get_time()
    seconds = rospy.get_time()
    while (seconds - start < timeout) and not rospy.is_shutdown():
      attached_objects = scene.get_attached_objects([box_name])
      is_attached = len(attached_objects.keys()) > 0
      is_known = box_name in scene.get_known_object_names()
      if (box_is_attached == is_attached) and (box_is_known == is_known):
        return True
      rospy.sleep(0.1)
      seconds = rospy.get_time()
    return False

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_name = "arm_with_torso"
move_group = moveit_commander.MoveGroupCommander(group_name)

box_pose = geometry_msgs.msg.PoseStamped()
box_pose.header.frame_id = "wrist_roll_link"
box_pose.pose.orientation.w = 1.0
box_pose.pose.position.x += 0.325
box_name = "box"
rospy.sleep(2)
scene.add_box(box_name, box_pose, size=(0.35, 0.2, 0.35))
print wait_for_state_update(box_name, scene, box_is_known=True)

grasping_group = 'gripper'
touch_links = robot.get_link_names(group=grasping_group)
touch_links.append("wrist_roll_link")
scene.attach_box(move_group.get_end_effector_link(), box_name, touch_links=touch_links)
print wait_for_state_update(box_name, scene, box_is_attached=True)


target_q = tf.transformations.quaternion_from_euler(0.0, 3.14 / 2.0, 0.0)
joint_goal = move_group.get_current_joint_values()

# #Joint pose for arm on the side 
joint_goal[0] = 0.0857168138027
joint_goal[1] = -1.58481834789
joint_goal[2] = 0.433445118112
joint_goal[3] = -1.53769913621
joint_goal[4] = 1.80451970107
joint_goal[5] = 2.0000173681
joint_goal[6] = 1.63692856729
joint_goal[7] = 2.86215073034

# Joint pose for arm in the font
# joint_goal[0] = 0.381449401379
# joint_goal[1] = -0.000215991931152
# joint_goal[2] = 0.244765899819
# joint_goal[3] = 0.000499797728465
# joint_goal[4] = 0.158174562521
# joint_goal[5] = -3.14188650916
# joint_goal[6] = -1.13804244101
# joint_goal[7] = 3.1386509554

move_group.set_joint_value_target(joint_goal)

# Maybe the robot planner didn't take it into account? Need to check move_group.launch
# Check https://github.com/ros-planning/moveit/pull/541 for detail problem due to 
# fetch_moveit_config/config/ompl_planner.yaml is modified
# Check if sampling in JointModelStateSpace is enforced for this group by user.
# This is done by setting 'enforce_joint_model_state_space' to 'true' for the desired group in ompl_planning.yaml.

# Some planning problems like orientation path constraints are represented in PoseModelStateSpace and sampled via IK.
# However consecutive IK solutions are not checked for proximity at the moment and sometimes happen to be flipped,
# leading to invalid trajectories. This workaround lets the user prevent this problem by forcing rejection sampling
# in JointModelStateSpace.

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

move_group.set_start_state(move_group.get_current_state())

move_group.set_planning_time(15)
myplan = move_group.plan()

move_group.execute(myplan)
# Calling `stop()` ensures that there is no residual movement
move_group.stop()

scene.remove_attached_object(move_group.get_end_effector_link(), name=box_name)
print wait_for_state_update(box_name, scene, box_is_attached=False)
scene.remove_world_object(box_name)
print wait_for_state_update(box_name, scene)