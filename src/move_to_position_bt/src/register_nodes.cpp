#include "behaviortree_ros2/plugins.hpp"
#include "move_to_position_bt/movetp_node.hpp"

BT_REGISTER_ROS_NODES(factory, params)
{
  factory.registerNodeType<move_to_position_bt::MoveToPositionNode>("MoveToPosition", params);
}