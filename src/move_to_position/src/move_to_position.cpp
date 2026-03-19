#include "move_to_position/move_to_position.hpp"

namespace move_to_position
{

MoveToPositionNode::MoveToPositionNode() : Node("move_to_position_node")
{
    target_pos_pub_ = this->create_publisher<geometry_msgs::msg::Pose>("target_position", 10);

    move_to_position_service_ = this->create_service<MoveToPosition>(
        "MoveToPosition_Service",
        std::bind(&MoveToPositionNode::handleService, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "MoveToPositionNode has been started.");
}

void MoveToPositionNode::handleService(
    const std::shared_ptr<MoveToPosition::Request> request,
    std::shared_ptr<MoveToPosition::Response> response)
{
    RCLCPP_INFO(
        this->get_logger(),
        "Received move request: [%.3f, %.3f, %.3f]",
        request->target_position.position.x,
        request->target_position.position.y,
        request->target_position.position.z);

    target_pos_pub_->publish(request->target_position);
    response->success = true;
}

}  // namespace move_to_position

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<move_to_position::MoveToPositionNode>());
    rclcpp::shutdown();
    return 0;
}