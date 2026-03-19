#include "behaviortree_ros2/plugins.hpp"
#include "move_to_position_bt/movetp_node.hpp"

BT_REGISTER_ROS_NODES(factory, params)
{
  BT::RosNodeParams fixed_params = params;
  fixed_params.default_port_value = "MoveToPosition_Service";
  factory.registerNodeType<move_to_position_bt::MoveToPositionNode>("MoveToPosition", fixed_params);
}