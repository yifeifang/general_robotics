#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_group_interface_tutorial_cpp_test");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  
  static const std::string PLANNING_GROUP = "arm_with_torso";
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  return 0;
}