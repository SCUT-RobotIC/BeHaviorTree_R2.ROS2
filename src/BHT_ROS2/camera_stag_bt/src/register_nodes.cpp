#include "behaviortree_ros2/plugins.hpp"
#include "camera_stag_bt/ali_ps_node.hpp"

BT_REGISTER_ROS_NODES(factory, params)
{
  factory.registerNodeType<camera_stag_bt::AliPsNode>("AliSp", params);
}