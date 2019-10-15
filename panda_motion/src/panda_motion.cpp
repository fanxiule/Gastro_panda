#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char **argv)
{
    //initialize ROS node
    ros::init(argc, argv, "panda_motion");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    //move group initialization
    static const std::string PLANNING_GROUP_ARM = "panda_arm";
    moveit::planning_interface::MoveGroupInterface move_group_arm(PLANNING_GROUP_ARM);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    const robot_state::JointModelGroup *arm_joint_model_group =
        move_group_arm.getCurrentState()->getJointModelGroup(PLANNING_GROUP_ARM);
    static const std::string PLANNING_GROUP_HAND = "hand";
    moveit::planning_interface::MoveGroupInterface move_group_hand(PLANNING_GROUP_HAND);
    const robot_state::JointModelGroup *hand_joint_model_group =
        move_group_hand.getCurrentState()->getJointModelGroup(PLANNING_GROUP_HAND);

    //initialize visualization
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0");
    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();
    ROS_INFO_NAMED("panda_motion", "Reference frame: %s", move_group_arm.getPlanningFrame().c_str());
    ROS_INFO_NAMED("panda_motion", "End effector link: %s", move_group_arm.getEndEffectorLink().c_str());
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window");

    //RECORD DEFAULT POSE
    geometry_msgs::Pose default_pose = move_group_arm.getCurrentPose().pose;

    //FAST APPROACH
    geometry_msgs::Pose fast_approach;
    fast_approach.orientation.x = -0.9238795;
    fast_approach.orientation.y = 0.3826834;
    fast_approach.orientation.z = 0;
    fast_approach.orientation.w = 0;
    fast_approach.position.x = 0.5;
    fast_approach.position.y = 0;
    fast_approach.position.z = 0.2;

    move_group_arm.setPoseTarget(fast_approach);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    bool success = (move_group_arm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("panda_motion", "Visualizing plan 1 (fast approach) %s", success ? "" : "FAILED");
    visual_tools.trigger();
    //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window");

    move_group_arm.execute(my_plan);
    ROS_INFO_NAMED("panda_motion", "Plan 1 exexuted");
    //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window");
    //END OF FAST APPROACH

    //OPENING GRIPPER
    moveit::core::RobotStatePtr hand_current_state = move_group_hand.getCurrentState();
    std::vector<double> hand_positions;
    hand_current_state->copyJointGroupPositions(hand_joint_model_group, hand_positions);
    hand_positions[0] = 0.04;
    move_group_hand.setJointValueTarget(hand_positions);

    success = (move_group_hand.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("panda_motion", "Visualizing plan 2 (open gripper) %s", success ? "" : "FAILED");
    visual_tools.trigger();
    //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window");

    move_group_hand.execute(my_plan);
    ROS_INFO_NAMED("panda_motion", "Plan 2 exexuted");
    //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window");
    //END OF OPENING GRIPPER

    //SLOW APPROACH
    geometry_msgs::Pose slow_approach = move_group_arm.getCurrentPose().pose;
    slow_approach.position.x += 0.05;
    move_group_arm.setMaxVelocityScalingFactor(0.01);
    move_group_arm.setPoseTarget(slow_approach);

    success = (move_group_arm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("panda_motion", "Visualizing plan 2 (slow approach) %s", success ? "" : "FAILED");
    // Visualize the plan in RViz
    visual_tools.trigger();
    //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window");

    move_group_arm.execute(my_plan);
    ROS_INFO_NAMED("panda_motion", "Plan 3 exexuted");
    //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window");
    //END OF SLOW APPROACH

    //CLOSING GRIPPER
    hand_current_state = move_group_hand.getCurrentState();
    hand_positions[0] = 0;
    move_group_hand.setJointValueTarget(hand_positions);

    success = (move_group_hand.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("panda_motion", "Visualizing plan 4 (close gripper) %s", success ? "" : "FAILED");
    visual_tools.trigger();
    //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window");

    move_group_hand.execute(my_plan);
    ROS_INFO_NAMED("panda_motion", "Plan 4 exexuted");
    //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window");
    //END OF CLOSING GRIPPER

    //LIFT PAN
    //set constraint for orientation
    moveit_msgs::OrientationConstraint orient_constraint_grip;
    orient_constraint_grip.link_name = "panda_link8";
    orient_constraint_grip.header.frame_id = "panda_link0";
    orient_constraint_grip.orientation.x = -0.9238795;
    orient_constraint_grip.orientation.y = 0.3826834;
    orient_constraint_grip.orientation.z = 0;
    orient_constraint_grip.orientation.w = 0;
    orient_constraint_grip.absolute_x_axis_tolerance = 0.1;
    orient_constraint_grip.absolute_y_axis_tolerance = 0.1;
    orient_constraint_grip.absolute_z_axis_tolerance = 0.1;
    orient_constraint_grip.weight = 1.0;
    // Now, set it as the path constraint for the group.
    moveit_msgs::Constraints grip_path_constraint;
    grip_path_constraint.orientation_constraints.push_back(orient_constraint_grip);
    move_group_arm.setPathConstraints(grip_path_constraint);
    ROS_INFO_NAMED("panda_motion", "Constraints set");

    geometry_msgs::Pose lift_pan;
    lift_pan.orientation.x = -0.9238795;
    lift_pan.orientation.y = 0.3826834;
    lift_pan.orientation.z = 0;
    lift_pan.orientation.w = 0;
    lift_pan.position.x = 0.3;
    lift_pan.position.y = 0;
    lift_pan.position.z = 0.45;
    move_group_arm.setPoseTarget(lift_pan);
    move_group_arm.setMaxVelocityScalingFactor(0.1);
    success = (move_group_arm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("panda_motion", "Visualizing plan 5 (lift pan) %s", success ? "" : "FAILED");
    visual_tools.trigger();
    //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window");

    move_group_arm.execute(my_plan);
    ROS_INFO_NAMED("panda_motion", "Plan 5 exexuted");
    //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window");
    //END OF LIFT PAN

    //MOVE PAN
    geometry_msgs::Pose move_pan;
    geometry_msgs::Pose current_arm_pose = move_group_arm.getCurrentPose().pose;

    std::vector<geometry_msgs::Pose> waypoints;
    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;

    move_pan.orientation.x = -0.9238795;
    move_pan.orientation.y = 0.3826834;
    move_pan.orientation.z = 0;
    move_pan.orientation.w = 0;
    move_pan.position.z = 0.45;

    for (double theta = 0; theta < 4 * M_PI; theta = theta + 0.1)
    {
        move_pan.position.x = 0.4 - 0.1 * cos(theta);
        move_pan.position.y = 0.1 * sin(theta);
        waypoints.push_back(move_pan);
    }

    double fraction = move_group_arm.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    my_plan.trajectory_ = trajectory;

    ROS_INFO_NAMED("panda_motion", "Visualizing plan 6 (move pan) (%.2f%% acheived)", fraction * 100.0);
    visual_tools.trigger();
    //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window");

    move_group_arm.execute(my_plan);
    ROS_INFO_NAMED("panda_motion", "Plan 6 exexuted");
    //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window");
    //END OF MOVE PAN

    //LOWER PAN
    geometry_msgs::Pose lower_pan;
    lower_pan.orientation.x = -0.9238795;
    lower_pan.orientation.y = 0.3826834;
    lower_pan.orientation.z = 0;
    lower_pan.orientation.w = 0;
    lower_pan.position.x = 0.55;
    lower_pan.position.y = -0.3;
    lower_pan.position.z = 0.2;

    move_group_arm.setPoseTarget(lower_pan);

    success = (move_group_arm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("panda_motion", "Visualizing plan 7 (lower pan) %s", success ? "" : "FAILED");
    visual_tools.trigger();
    //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window");

    move_group_arm.execute(my_plan);
    ROS_INFO_NAMED("panda_motion", "Plan 7 exexuted");
    //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window");
    //END OF LOWER PAN

    //OPENING GRIPPER
    hand_current_state = move_group_hand.getCurrentState();
    hand_positions[0] = 0.04;
    move_group_hand.setJointValueTarget(hand_positions);

    success = (move_group_hand.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("panda_motion", "Visualizing plan 8 (open gripper) %s", success ? "" : "FAILED");
    visual_tools.trigger();
    //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window");

    move_group_hand.execute(my_plan);
    ROS_INFO_NAMED("panda_motion", "Plan 8 exexuted");
    //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window");
    //END OF OPENING GRIPPER

    //SLOW RETREAT
    geometry_msgs::Pose slow_retreat = move_group_arm.getCurrentPose().pose;
    slow_retreat.position.x -= 0.05;
    move_group_arm.setMaxVelocityScalingFactor(0.01);
    move_group_arm.setPoseTarget(slow_retreat);

    success = (move_group_arm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("panda_motion", "Visualizing plan 9 (slow retreat) %s", success ? "" : "FAILED");
    visual_tools.trigger();
    //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window");

    move_group_arm.execute(my_plan);
    ROS_INFO_NAMED("panda_motion", "Plan 9 exexuted");
    //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window");
    //END OF SLOW RETREAT

    //CLOSING GRIPPER
    hand_current_state = move_group_hand.getCurrentState();
    //hand_current_state->copyJointGroupPositions(hand_joint_model_group, hand_positions);
    hand_positions[0] = 0;
    move_group_hand.setJointValueTarget(hand_positions);

    success = (move_group_hand.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("panda_motion", "Visualizing plan 10 (close gripper) %s", success ? "" : "FAILED");
    visual_tools.trigger();
    //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window");

    move_group_hand.execute(my_plan);
    ROS_INFO_NAMED("panda_motion", "Plan 10 exexuted");
    //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window");
    //END OF CLOSING GRIPPER

    //RETURN TO DEFAULT
    move_group_arm.clearPathConstraints();
    move_group_arm.setPoseTarget(default_pose);
    move_group_arm.setMaxVelocityScalingFactor(1);
    success = (move_group_arm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("panda_motion", "Visualizing plan 11 (return to default) %s", success ? "" : "FAILED");
    visual_tools.trigger();
    //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window");

    move_group_arm.execute(my_plan);
    ROS_INFO_NAMED("panda_motion", "Plan 11 exexuted");
    //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window");
    //RETURN TO DEFAULT

    ros::shutdown();
    return 0;
}
