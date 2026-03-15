#include "behaviortree_ros2/plugins.hpp"
#include "yolo_spearhead_see_bt/yolo_spearhead_see_node.hpp"

BT_REGISTER_ROS_NODES(factory, params)
{
  factory.registerNodeType<yolo_spearhead_see_bt::YOLOSpearheadSeeNode>("FetchSp", params);
}