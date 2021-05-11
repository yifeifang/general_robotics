#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "dewey_motionplan");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    static const std::string PLANNING_GROUP = "arm_with_torso";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    geometry_msgs::Pose target_pose1;
    target_pose1.orientation.x = 0.0;
    target_pose1.orientation.y = 0.706825181105366;
    target_pose1.orientation.z = 0.0;
    target_pose1.orientation.w = 0.7073882691671998;
    target_pose1.position.x = 0.3;
    target_pose1.position.y = 0.0;
    target_pose1.position.z = 0.8;

    move_group.setPoseTarget(target_pose1);

    moveit_msgs::OrientationConstraint ocm;
    ocm.link_name = "wrist_roll_link";
    ocm.header.frame_id = "base_link";
    ocm.orientation.x = 0.0;
    ocm.orientation.y = 0.706825181105366;
    ocm.orientation.z = 0.0;
    ocm.orientation.w = 0.7073882691671998;
    ocm.absolute_x_axis_tolerance = 0.1;
    ocm.absolute_y_axis_tolerance = 0.1;
    ocm.absolute_z_axis_tolerance = 0.1;
    ocm.weight = 1.0;

    moveit_msgs::Constraints test_constraints;
    test_constraints.orientation_constraints.push_back(ocm);
    move_group.setPathConstraints(test_constraints);

    robot_state::RobotState start_state(*move_group.getCurrentState());
    // geometry_msgs::Pose start_pose2;
    // start_pose2.orientation.w = 1.0;
    // start_pose2.position.x = 0.3;
    // start_pose2.position.y = 0.0;
    // start_pose2.position.z = 0.5;
    // start_state.setFromIK(joint_model_group, start_pose2);
    move_group.setStartState(start_state);

    move_group.setPoseTarget(target_pose1);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
    move_group.move();
    return 0;
}