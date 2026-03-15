#include "move_to_position/move_to_position.hpp"

namespace move_to_position
{
    MoveToPositionNode::MoveToPositionNode() : Node("move_to_position_node")
    {
        // 发布目标位置
        target_pos_pub = this->create_publisher<geometry_msgs::msg::Pose>("target_position", 10);
        
        // 创建服务
        move_to_position_service = this->create_service<MoveToPosition>(
            "move_to_position",
            std::bind(&MoveToPositionNode::handle_service, this, std::placeholders::_1, std::placeholders::_2)
        );

        RCLCPP_INFO(this->get_logger(), "MoveToPositionNode has been started.");

        void handle_service(const std::shared_ptr<MoveToPosition::Request> request, std::shared_ptr<MoveToPosition::Response> response)
        {
            RCLCPP_INFO(this->get_logger(), "Received request to move to position: [%.2f, %.2f, %.2f]", 
                        request->target_position.position.x, 
                        request->target_position.position.y, 
                        request->target_position.position.z);

            // 发布目标位置
            target_pos_pub->publish(request->target_position);

            // 设置响应
            response->success = true;
        }
    }
}