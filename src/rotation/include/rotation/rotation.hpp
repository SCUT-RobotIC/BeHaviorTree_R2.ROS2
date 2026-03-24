#pragma once

#include <memory>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "movetp_interface/srv/move_to_position.hpp"
#include "rclcpp/rclcpp.hpp"

namespace move_to_position
{

using MoveToPosition = movetp_interface::srv::MoveToPosition;

class MoveToPositionNode : public rclcpp::Node
{
public:
    MoveToPositionNode();

private:
    void handleService(const std::shared_ptr<MoveToPosition::Request> request,
                                         std::shared_ptr<MoveToPosition::Response> response);

    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr target_pos_pub_;
    rclcpp::Service<MoveToPosition>::SharedPtr move_to_position_service_;
};

}  // namespace move_to_position