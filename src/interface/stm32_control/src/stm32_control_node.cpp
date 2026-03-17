#include "stm32_control/stm32_control_node.hpp"

Stm32ControlNode::Stm32ControlNode() : Node("stm32_control_node") {
    load_parameters();

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // register built-in flow definitions before creating handler
    stm32_control::register_builtin_flows();

    // pick flow based on parameter (fall back to default if missing)
    std::shared_ptr<stm32_control::Flow> selected_flow =
        stm32_control::FlowRegistry::instance().create(flow_name_);
    if (!selected_flow) {
        RCLCPP_WARN(this->get_logger(), "flow '%s' not found, using default grab_tip",
                    flow_name_.c_str());
        selected_flow = stm32_control::FlowRegistry::instance().create("grab_tip");
    }

    action_handler_ = std::make_unique<ActionHandler>(
      this->get_logger(),
      this->get_clock(),
      *tf_buffer_,
      odom_frame_,
      base_frame_,
      use_pose_topic_for_current_pose_,
      selected_flow
    );

    // Publisher for outgoing packets to communication node
    tx_pub_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>(stm32_write_topic_, 10);

    // Subscriber for incoming packets from communication node
    rx_sub_ = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
        stm32_read_topic_, 10,
        std::bind(&Stm32ControlNode::on_packet_received, this, std::placeholders::_1)
    );

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
