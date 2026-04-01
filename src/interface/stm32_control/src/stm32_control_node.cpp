#include "stm32_control/stm32_control_node.hpp"

/**
 * @file stm32_control_node.cpp
 * @brief 实现 Stm32ControlNode 构造与主入口
 */

/**
 * @brief 构造函数：加载参数、初始化 TF、发布/订阅并启动周期定时器
 */
Stm32ControlNode::Stm32ControlNode() : Node("stm32_control_node") {
    load_parameters();

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    setup_ros_communications();

    // Timer for high frequency control loop
    send_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / frequency_)),
        std::bind(&Stm32ControlNode::send_timer_callback, this)
    );

    RCLCPP_INFO(this->get_logger(), "Stm32 Control Node initialized.");
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Stm32ControlNode>());
    rclcpp::shutdown();
    return 0;
}
