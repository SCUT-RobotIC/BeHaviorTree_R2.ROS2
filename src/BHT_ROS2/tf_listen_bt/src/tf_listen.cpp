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

	node_->declare_parameter<double>("tf_listen_xy_error", 0.2);
	node_->declare_parameter<double>("tf_listen_timeout", 0.0);
	node_->declare_parameter<std::string>("tf_listen_target_frame", "lidar_link");
	node_->declare_parameter<std::string>("tf_listen_source_frame", "lidar_odom");
	

	node_->get_parameter("tf_listen_xy_error", xy_error_);
	node_->get_parameter("tf_listen_timeout", default_timeout_);
	node_->get_parameter("tf_listen_target_frame", default_target_frame_);
	node_->get_parameter("tf_listen_source_frame", default_source_frame_);
	

	tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
	tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

BT::PortsList TfListenNode::providedPorts()
{
	return {
		BT::InputPort<geometry_msgs::msg::Pose>("Target_Position"),
		BT::InputPort<std::string>("target_frame", "TF target frame override"),
		BT::InputPort<std::string>("source_frame", "TF source frame override"),
		BT::InputPort<double>("timeout", "Lookup timeout seconds override"),
	};
}

BT::NodeStatus TfListenNode::tick()
{
	auto target_pose = getInput<geometry_msgs::msg::Pose>("Target_Position");
	if(!target_pose)
	{
		return BT::NodeStatus::FAILURE;
	}

	std::string target_frame = default_target_frame_;
	if(auto input_target_frame = getInput<std::string>("target_frame"))
	{
		target_frame = input_target_frame.value();
	}

	std::string source_frame = default_source_frame_;
	if(auto input_source_frame = getInput<std::string>("source_frame"))
	{
		source_frame = input_source_frame.value();
	}

	double timeout = default_timeout_;
	if(auto input_timeout = getInput<double>("timeout"))
	{
		timeout = input_timeout.value();
	}

	try
	{
		geometry_msgs::msg::TransformStamped transform;
		if(timeout > 0.0)
		{
			transform = tf_buffer_->lookupTransform(
				target_frame, source_frame, tf2::TimePointZero,
				tf2::durationFromSec(timeout));
		}
		else
		{
			transform = tf_buffer_->lookupTransform(
				target_frame, source_frame, tf2::TimePointZero);
		}

		geometry_msgs::msg::Pose Position_Now;
		Position_Now.position.x = transform.transform.translation.x;
		Position_Now.position.y = transform.transform.translation.y;
		Position_Now.position.z = transform.transform.translation.z;
		Position_Now.orientation = transform.transform.rotation;

		const bool within_xy_error =
			std::abs(Position_Now.position.x - target_pose.value().position.x) <= xy_error_ &&
			std::abs(Position_Now.position.y - target_pose.value().position.y) <= xy_error_;

		return within_xy_error ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
	}
	catch(const tf2::TransformException&)
	{
		return BT::NodeStatus::FAILURE;
	}
}

}  // namespace tf_listen_bt
