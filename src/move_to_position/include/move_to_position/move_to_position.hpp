#pragma once

#include <memory>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "movetp_interface/action/move_to_position.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16.hpp"
#include <atomic>
namespace move_to_position
{

using MoveToPosition = movetp_interface::action::MoveToPosition;

class MoveToPositionNode : public rclcpp::Node
{
public:
    MoveToPositionNode();

private:

    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const MoveToPosition::Goal> goal);
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveToPosition>> goal_handle);
    void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveToPosition>> goal_handle);
    void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveToPosition>> goal_handle);

    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr ACK_sub;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr target_pos_pub_;
    rclcpp_action::Server<MoveToPosition>::SharedPtr move_to_position_action_server_;
    
    std::atomic<int> ack_{0};
};

}  // namespace move_to_position