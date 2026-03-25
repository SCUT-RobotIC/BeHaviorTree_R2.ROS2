#include "rotation/rotation.hpp"
#include <thread>

namespace rotation
{

RotationNode::RotationNode() : Node("rotation_node")
{
	direction_pub_ = this->create_publisher<std_msgs::msg::Int16>("rotation_goal", 10);
	ACK_sub = this->create_subscription<std_msgs::msg::Int16>(
		"ack", 10, [this](const std_msgs::msg::Int16::SharedPtr msg) {
			ack_.store(msg->data);
			RCLCPP_INFO(this->get_logger(), "Received ACK: %d", ack_.load());
		});

	rotation_action_server_ = rclcpp_action::create_server<Rotation>(
		this,
		"rotation",
		std::bind(&RotationNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
		std::bind(&RotationNode::handle_cancel, this, std::placeholders::_1),
		std::bind(&RotationNode::handle_accepted, this, std::placeholders::_1)
	);
	RCLCPP_INFO(this->get_logger(), "RotationNode (action server) has been started.");
}

rclcpp_action::GoalResponse RotationNode::handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Rotation::Goal> goal)
{
	(void)uuid;
	RCLCPP_INFO(this->get_logger(), "Received goal request: [%d]", goal->direction);
	return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse RotationNode::handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<Rotation>> goal_handle)
{
	RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
	return rclcpp_action::CancelResponse::ACCEPT;
}

void RotationNode::handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<Rotation>> goal_handle)
{
	std::thread{std::bind(&RotationNode::execute, this, goal_handle)}.detach();
}

void RotationNode::execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<Rotation>> goal_handle)
{
	const auto goal = goal_handle->get_goal();
	auto result = std::make_shared<Rotation::Result>();
	std_msgs::msg::Int16 msg;
	msg.data = goal->direction;
	rclcpp::Rate rate(10); // 10Hz
	while(ack_.load() != 2 && rclcpp::ok()){
		direction_pub_->publish(msg);
		rate.sleep();
	}
	result->success = true;
	goal_handle->succeed(result);
	RCLCPP_INFO(this->get_logger(), "Goal succeeded");
}

} // namespace rotation

int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<rotation::RotationNode>());
	rclcpp::shutdown();
	return 0;
}
