#include<rclcpp/rclcpp.hpp>
#include<geometry_msgs/msg/pose_stamped.hpp>
#include "movetp_interface/srv/MoveToPosition.hpp"

using MoveToPosition = movetp_interface::srv::MoveToPosition;

namespace move_to_position
{

class MoveToPositionNode : public rclcpp::Node
{
    public:
        explicit MoveToPositionNode(const rclcpp::NodeOptions& options);
    private:
        std::shared_ptr<rclcpp::Node> node_;
        rclcpp::Publisher<geometry_msgs::msg::posestamped>::SharedPtr target_pos;
        rclcpp::Service<MoveToPosition>::SharedPtr move_to_position_service;
        void handle_service(const std::shared_ptr<MoveToPosition::Request> request, std::shared_ptr<MoveToPosition::Response> response);
}
}