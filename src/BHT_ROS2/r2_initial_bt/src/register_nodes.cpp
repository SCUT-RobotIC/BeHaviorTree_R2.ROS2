#include "behaviortree_ros2/plugins.hpp"
#include "r2_initial/subscribe_once_node.hpp"

BT_REGISTER_ROS_NODES(factory, params)
{
  factory.registerNodeType<r2_initial::R2InitialNode>("R2Initial", params);
}
