#include "behaviortree_ros2/plugins.hpp"
#include "yolo_spearhead_see_bt/yolo_spearhead_see_node.hpp"

BT_REGISTER_ROS_NODES(factory, params)
{
  BT::RosNodeParams fixed_params = params;
  fixed_params.default_port_value = "FetchSp_Action";
  factory.registerNodeType<yolo_spearhead_see_bt::YoloNode>("FetchSp", fixed_params);
}
