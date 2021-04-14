#!/usr/bin/env python
# license removed for brevity

import rospy
# Brings in the SimpleActionClient
import actionlib
# Brings in the .action file and messages used by the move base action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def get_goal(x,y,z,qx,qy,qz,qw):
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
  # Select position based on the "map" coordinate frame 
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.position.z = z
  # Select orientation w.r.t. map frame
    goal.target_pose.pose.orientation.x = qx
    goal.target_pose.pose.orientation.y = qy
    goal.target_pose.pose.orientation.z = qz
    goal.target_pose.pose.orientation.w = qw

    return goal

# takes a MoveBaseGoal object
def movebase_client(goal):

   # Create an action client called "move_base" with action definition file "MoveBaseAction"
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
 
   # Waits until the action server has started up and started listening for goals.
    client.wait_for_server()

   # Sends the goal to the action server.
    client.send_goal(goal)
   # Waits for the server to finish performing the action.
    wait = client.wait_for_result()
   # If the result doesn't arrive, assume the Server is not available
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
    # Result of executing the action
        return client.get_result()   

# If the python node is executed as main process (sourced directly)
if __name__ == '__main__':
    try:
       # Initializes a rospy node to let the SimpleActionClient publish and subscribe
        rospy.init_node('planning_goal')

        # Creates a new goal with the MoveBaseGoal constructor
        goal_list = []
        # # table
        # goal_list.append(get_goal(3.469, 3.349, 0.000, -0.0, -0.0, 0.212, 0.977))   
        # # start pose
        # goal_list.append(get_goal(-0.459, -0.880, 0.000, 0.0, 0.0, -0.064, 0.998))
        # in door init
        goal_list.append(get_goal(-5.554, 1.341, 0.000, 0.0, 0.0, 0.730, 0.684))
        # in door final
        goal_list.append(get_goal(-5.583, -0.682, 0.000, 0.0, 0.0, -0.736, 0.677))
        # out door init
        goal_list.append(get_goal(-5.444, -0.462, 0.000, 0.0, 0.0, 0.668, 0.744))
        # out door final
        goal_list.append(get_goal(-5.356, 1.185, 0.000, 0.0, 0.0, 0.682, 0.731))
        # start pose
        goal_list.append(get_goal(-0.459, -0.880, 0.000, 0.0, 0.0, -0.064, 0.998))

        for goal in goal_list:
          result = movebase_client(goal)
          if result:
              rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
