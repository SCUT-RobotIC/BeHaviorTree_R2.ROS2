#include "tf_listen_bt/tf_listen.hpp"

#include <cmath>

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
		BT::InputPort<geometry_msgs::msg::Pose>("Target_Position"),
		BT::InputPort<std::string>("target_frame", "lidar_link", "TF target frame"),
		BT::InputPort<std::string>("source_frame", "lidar_odom", "TF source frame"),
		BT::InputPort<double>("timeout", 0.0, "Lookup timeout seconds"),
	};
}

BT::NodeStatus TfListenNode::tick()
{
	auto target_pose = getInput<geometry_msgs::msg::Pose>("Target_Position");
	if(!target_pose)
	{
		return BT::NodeStatus::FAILURE;
	}

	auto target_frame = getInput<std::string>("target_frame");
	if(!target_frame)
	{
		return BT::NodeStatus::FAILURE;
	}

	auto source_frame = getInput<std::string>("source_frame");
	if(!source_frame)
	{
		return BT::NodeStatus::FAILURE;
	}

	auto timeout = getInput<double>("timeout");
	if(!timeout)
	{
		return BT::NodeStatus::FAILURE;
	}

	try
	{
		geometry_msgs::msg::TransformStamped transform;
		if(timeout.value() > 0.0)
		{
			transform = tf_buffer_->lookupTransform(
				target_frame.value(), source_frame.value(), tf2::TimePointZero,
				tf2::durationFromSec(timeout.value()));
		}
		else
		{
			transform = tf_buffer_->lookupTransform(
				target_frame.value(), source_frame.value(), tf2::TimePointZero);
		}

		geometry_msgs::msg::Pose Position_Now;
		Position_Now.position.x = transform.transform.translation.x;
		Position_Now.position.y = transform.transform.translation.y;
		Position_Now.position.z = transform.transform.translation.z;
		Position_Now.orientation = transform.transform.rotation;

		const bool within_xy_error =
			std::abs(Position_Now.position.x - target_pose.value().position.x) <= error &&
			std::abs(Position_Now.position.y - target_pose.value().position.y) <= error;

		return within_xy_error ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
	}
	catch(const tf2::TransformException&)
	{
		return BT::NodeStatus::FAILURE;
	}
}

}  // namespace tf_listen_bt
