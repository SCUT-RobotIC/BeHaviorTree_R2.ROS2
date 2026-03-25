#include "behaviortree_ros2/plugins.hpp"
#include "rotation_bt/rotation_node.hpp"

BT_REGISTER_ROS_NODES(factory, params)
{
  BT::RosNodeParams fixed_params = params;
  fixed_params.default_port_value = "Rotation_Action";
  factory.registerNodeType<rotation_bt::RotationNode>("Rotation", fixed_params);
}