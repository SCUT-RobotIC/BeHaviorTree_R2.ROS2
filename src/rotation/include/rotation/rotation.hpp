#pragma once

#include <memory>
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/int16.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rotation_interface/action/rotation.hpp"
#include <atomic>

namespace rotation
{
    
using Rotation = rotation_interface::action::Rotation;
class RotationNode : public rclcpp::Node
{
public:
    RotationNode();

private:
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Rotation::Goal> goal);
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<Rotation>> goal_handle);
    void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<Rotation>> goal_handle);
    void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<Rotation>> goal_handle);

    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr ACK_sub;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr direction_pub_;
    rclcpp_action::Server<Rotation>::SharedPtr rotation_action_server_;
    
    std::atomic<int> ack_{0};
    std::condition_variable ack_cv_;
    std::mutex ack_mutex_;
    std::shared_ptr<rclcpp::TimerBase> timer;
};

} // namespace rotation