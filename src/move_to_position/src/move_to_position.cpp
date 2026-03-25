#include "move_to_position/move_to_position.hpp"
#include <thread>
#include "rclcpp_action/rclcpp_action.hpp"
#include <condition_variable>
#include <mutex>
namespace move_to_position
{
MoveToPositionNode::MoveToPositionNode() : Node("move_to_position_node")
{
    target_position_pub_ = this->create_publisher<geometry_msgs::msg::Point>("target_position", 10);
    ACK_sub = this->create_subscription<std_msgs::msg::Int16>(
        "ack", 10, [this](const std_msgs::msg::Int16::SharedPtr msg) {
            ack_.store(msg->data);
            RCLCPP_INFO(this->get_logger(), "Received ACK: %d", ack_.load());
            if (ack_.load() == 2) {
                std::lock_guard<std::mutex> lock(ack_mutex_);
                ack_cv_.notify_all();
            }
        });
    using namespace std::placeholders;
    move_to_position_action_server_ = rclcpp_action::create_server<MoveToPosition>(
        this,
        "MoveToPosition_Action",
        std::bind(&MoveToPositionNode::handle_goal, this, _1, _2),
        std::bind(&MoveToPositionNode::handle_cancel, this, _1),
        std::bind(&MoveToPositionNode::handle_accepted, this, _1)
    );
    RCLCPP_INFO(this->get_logger(), "MoveToPositionNode (action server) has been started.");
}


rclcpp_action::GoalResponse MoveToPositionNode::handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const MoveToPosition::Goal> goal)
{
    (void)uuid;
    RCLCPP_INFO(this->get_logger(), "Received goal request: [%.3f, %.3f, %.3f]", goal->goal.x, goal->goal.y, goal->goal.z);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MoveToPositionNode::handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveToPosition>> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void MoveToPositionNode::handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveToPosition>> goal_handle)
{
    std::thread{std::bind(&MoveToPositionNode::execute, this, goal_handle)}.detach();
}

void MoveToPositionNode::execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveToPosition>> goal_handle)
{
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<MoveToPosition::Result>();
    geometry_msgs::msg::Point target_position;
    target_position.x = goal->goal.x;
    target_position.y = goal->goal.y;
    target_position.z = goal->goal.z;

    // 用定时器定时发布目标点，收到ack后停止定时器
    timer = this->create_wall_timer(
        std::chrono::milliseconds(50),
        [this, &timer, target_position]() mutable {
            if (ack_ != 2) {
                target_position_pub_->publish(target_position);
            } else {
                timer->cancel();
            }
        }
    );

    // 用条件变量等待ack_变为2
    std::unique_lock<std::mutex> lock(ack_mutex_);
    ack_cv_.wait(lock, [this]() { return ack_ == 2 || !rclcpp::ok(); });

    result->success = true;
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Goal succeeded");
}

}  // namespace move_to_position


int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<move_to_position::MoveToPositionNode>());
    rclcpp::shutdown();
    return 0;
}