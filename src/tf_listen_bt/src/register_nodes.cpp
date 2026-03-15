#include "behaviortree_ros2/plugins.hpp"
#include "tf_listen_bt/tf_listen.hpp"

BT_REGISTER_ROS_NODES(factory, params)
{
	factory.registerNodeType<tf_listen_bt::TfListenNode>("TfListen", params);
}
