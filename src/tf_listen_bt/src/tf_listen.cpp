#include "tf_listen_bt/tf_listen.hpp"

#include "tf2/exceptions.h"

namespace tf_listen_bt
{

TfListenNode::TfListenNode(const std::string& name, const BT::NodeConfig& config,
													 const BT::RosNodeParams& params)
: BT::SyncActionNode(name, config)
{
	node_ = params.nh.lock();
	if(!node_)
	{
		throw std::runtime_error("TfListenNode requires a valid rclcpp::Node");
	}

	tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
	tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

BT::PortsList TfListenNode::providedPorts()
{
	return {
		BT::InputPort<double>("Position1"),
		BT::InputPort<double>("timeout", 0.0, "Lookup timeout seconds"),
		BT::OutputPort<geometry_msgs::msg::Pose>("Position2")
	};
}

BT::NodeStatus TfListenNode::tick()
{
	auto timeout = getInput<double>("timeout");

	try
	{
		geometry_msgs::msg::TransformStamped transform;
		if(timeout.value() > 0.0)
		{
			transform = tf_buffer_->lookupTransform(
				lidar_link.value(), lidar_odom.value(), tf2::TimePointZero,
				tf2::durationFromSec(timeout.value()));
		}
		else
		{
			transform = tf_buffer_->lookupTransform(
				lidar_link.value(), lidar_odom.value(), tf2::TimePointZero);
		}

		if (transform.transform.translation.x - Position1.x < error &&
			transform.transform.translation.y - Position1.y < error &&)
		{
			return BT::NodeStatus::SUCCESS;
		}
		
		output_pose_ = geometry_msgs::msg::Pose();
		output_pose_.position.x = 0.0;
		output_pose_.position.y = 0.0;
		output_pose_.position.z = 0.0;
		output_pose_.orientation.w = 1.0;

		setOutput("Position2", output_pose_);
		return BT::NodeStatus::SUCCESS;
	}
	catch(const tf2::TransformException&)
	{
		return BT::NodeStatus::FAILURE;
	}
}

}  // namespace tf_listen_bt
