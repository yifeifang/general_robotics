#! /usr/bin/env python

import actionlib
import control_msgs.msg
import rospy
from sensor_msgs.msg import JointState

CLOSED_POS = 0.0  # The position for a fully-closed gripper (meters).
OPENED_POS = 0.10  # The position for a fully-open gripper (meters).
ACTION_SERVER = 'gripper_controller/gripper_action'


class Gripper(object):
    """Gripper controls the robot's gripper.
    """
    MIN_EFFORT = 35  # Min grasp force, in Newtons
    MAX_EFFORT = 100  # Max grasp force, in Newtons

    def __init__(self):
        self._client = actionlib.SimpleActionClient(ACTION_SERVER, control_msgs.msg.GripperCommandAction)
        self._client.wait_for_server(rospy.Duration(10))

    def open(self):
        """Opens the gripper.
        """
        goal = control_msgs.msg.GripperCommandGoal()
        goal.command.position = OPENED_POS
        self._client.send_goal_and_wait(goal, rospy.Duration(10))

    def close(self, max_effort=MAX_EFFORT):
        """Closes the gripper.

        Args:
            max_effort: The maximum effort, in Newtons, to use. Note that this
                should not be less than 35N, or else the gripper may not close.
        """
        goal = control_msgs.msg.GripperCommandGoal()
        goal.command.position = CLOSED_POS
        goal.command.max_effort = max_effort
        self._client.send_goal_and_wait(goal, rospy.Duration(10))

    def get_state(self, max_tries=20):
        """
        Queries the current joint angle of the gripper
        :param max_tries: the maximum number of times to try to retrieve the joint states
        :return: the gripper finger position in meters, or None if it couldn't be retrieved
        """
        i = 0
        while i < max_tries:
            joint_states = rospy.wait_for_message("/joint_states", JointState, timeout=rospy.Duration(1.0))
            # In simulation, all joints are published in a single message. On the robot, gripper is
            # separate.
            if "l_gripper_finger_joint" in joint_states.name:
                # We'll assume that both fingers are in same position
                index = joint_states.name.index("l_gripper_finger_joint")
                return joint_states.position[index]
        return None
