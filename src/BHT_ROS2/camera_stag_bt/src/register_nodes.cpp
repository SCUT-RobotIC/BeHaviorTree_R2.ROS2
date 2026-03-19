#include "behaviortree_ros2/plugins.hpp"
#include "camera_stag_bt/ali_ps_node.hpp"

BT_REGISTER_ROS_NODES(factory, params)
{
    BT::RosNodeParams fixed_params = params;
  fixed_params.default_port_value = "AliSp_Action";
  factory.registerNodeType<yolo_spearhead_see_bt::YoloNode>("AliSp", fixed_params);
}