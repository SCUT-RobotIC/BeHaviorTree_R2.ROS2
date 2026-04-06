#include "behaviortree_ros2/plugins.hpp"
#include "robot_arm_bt/robot_arm_publish_node.hpp"
#include "robot_arm_bt/robot_arm_solve_node.hpp"

BT_REGISTER_ROS_NODES(factory, params)
{
  BT::RosNodeParams fixed_params = params;
  fixed_params.default_port_value = "solve_arm_ik";
  factory.registerNodeType<robot_arm_bt::RobotArmSolveNode>("RobotArmSolve", fixed_params);
  factory.registerNodeType<robot_arm_bt::RobotArmPublishNode>("RobotArmPublish", params);
}