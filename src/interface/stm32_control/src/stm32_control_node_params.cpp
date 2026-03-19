#include "stm32_control/stm32_control_node.hpp"

void Stm32ControlNode::load_parameters() {
    frequency_ = this->declare_parameter<double>("frequency", 50.0);
    odom_frame_ = this->declare_parameter<std::string>("odom_name", "odom");
    base_frame_ = this->declare_parameter<std::string>("base_link_name", "base_link");
    goal_pose_topic_ = this->declare_parameter<std::string>("goal_pose_topic_name", "/goal_pose");
    stag_center_topic_ = this->declare_parameter<std::string>("stag_center_topic_name", "/stag_center");
    stag_detected_topic_ = this->declare_parameter<std::string>("stag_detected_topic_name", "/stag_detected");
    target_heading_topic_ = this->declare_parameter<std::string>("target_heading_topic_name", "/target_heading");
    stair_direction_topic_ = this->declare_parameter<std::string>("stair_direction_topic_name", "/stair_direction");
    arm_angles_topic_ = this->declare_parameter<std::string>("arm_angles_topic_name", "/arm_angles");
    yolo_offsets_topic_ = this->declare_parameter<std::string>("yolo_offsets_topic_name", "/yolo_detection_offsets");
    action_code_topic_ = this->declare_parameter<std::string>("action_code_topic_name", "/stm32/action_code");
    stm32_read_topic_ = this->declare_parameter<std::string>("stm32_read_topic", "stm32/read");
    stm32_write_topic_ = this->declare_parameter<std::string>("stm32_write_topic", "stm32/write");
    publish_ack_flag_ = this->declare_parameter<bool>("publish_ack_flag", false);
    ack_flag_topic_ = this->declare_parameter<std::string>("ack_flag_topic_name", "/stm32/ack_flag");
    use_pose_topic_for_current_pose_ = this->declare_parameter<bool>("use_pose_topic_for_current_pose", false);
    current_pose_topic_ = this->declare_parameter<std::string>("current_pose_topic_name", "/glim_ros/pose_corrected");
}

int16_t Stm32ControlNode::clamp_to_i16(double value) {
    constexpr double min_v = -32768.0;
    constexpr double max_v = 32767.0;
    return static_cast<int16_t>(std::clamp(std::round(value), min_v, max_v));
}

void Stm32ControlNode::setup_ros_communications() {
    // Target Subs
    auto target_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();

    action_code_sub_ = this->create_subscription<std_msgs::msg::Int16>(
        action_code_topic_, target_qos,
        [this](const std_msgs::msg::Int16::SharedPtr msg) {
            latest_action_code_.store(msg->data, std::memory_order_relaxed);
            action_code_updated_.store(true, std::memory_order_release);
        }
    );

    if (publish_ack_flag_) {
        ack_flag_pub_ = this->create_publisher<std_msgs::msg::Int16>(ack_flag_topic_, 10);
        RCLCPP_INFO(this->get_logger(), "ACK flag publish enabled on topic '%s'", ack_flag_topic_.c_str());
    }

    target_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
        goal_pose_topic_, target_qos,
        [this](const geometry_msgs::msg::Point::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(target_mutex_);
            this->target_data_.x_target = clamp_to_i16(std::round(msg->x * 1000.0));
            this->target_data_.y_target = clamp_to_i16(std::round(msg->y * 1000.0));
            this->target_data_.last_update_time = this->now();
            RCLCPP_INFO(this->get_logger(), "Received target: (%.2f, %.2f)", msg->x, msg->y);
        }
    );

    target_heading_sub_ = this->create_subscription<std_msgs::msg::Int16>(
        target_heading_topic_, 10,
        [this](const std_msgs::msg::Int16::SharedPtr msg) {
            const int16_t heading = std::clamp<int16_t>(msg->data, 0, 3);
            std::lock_guard<std::mutex> lock(target_mutex_);
            this->target_data_.heading = heading;
            this->target_data_.last_update_time = this->now();
        }
    );

    stair_direction_sub_ = this->create_subscription<std_msgs::msg::Int16>(
        stair_direction_topic_, 10,
        [this](const std_msgs::msg::Int16::SharedPtr msg) {
            const int16_t direction = std::clamp<int16_t>(msg->data, 0, 1);
            std::lock_guard<std::mutex> lock(target_mutex_);
            this->target_data_.stair_direction = direction;
            this->target_data_.last_update_time = this->now();
        }
    );

    // STAG
    stag_center_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
        stag_center_topic_, 10,
        [this](const geometry_msgs::msg::Point::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(stag_mutex_);
            this->stag_data_.x = static_cast<int16_t>(msg->x);
            this->stag_data_.y = static_cast<int16_t>(msg->y);
            this->stag_data_.last_update_time = this->now();
        }
    );

    stag_detected_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        stag_detected_topic_, 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(stag_mutex_);
            this->stag_data_.detected = msg->data;
        }
    );

    // Arm data comes from topic only.
    arm_angles_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        arm_angles_topic_, 10,
        [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(arm_mutex_);
            if (msg->data.size() >= 4) {
                this->arm_data_.joint1 = static_cast<int16_t>(std::round(msg->data[0] * 10.0));
                this->arm_data_.joint2 = static_cast<int16_t>(std::round(msg->data[1] * 10.0));
                this->arm_data_.joint3 = static_cast<int16_t>(std::round(msg->data[2] * 10.0));
                this->arm_data_.yaw = static_cast<int16_t>(std::round(msg->data[3] * 10.0));
                this->arm_data_.last_update_time = this->now();
            }
        }
    );
    RCLCPP_INFO(this->get_logger(), "Arm angle source: topic '%s'", arm_angles_topic_.c_str());

    // YOLO
    yolo_offsets_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        yolo_offsets_topic_, 10,
        [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(yolo_mutex_);
            if (msg->data.size() >= 2) {
                this->yolo_data_.x_offset = static_cast<int16_t>(std::round(msg->data[0]));
                this->yolo_data_.y_offset = static_cast<int16_t>(std::round(msg->data[1]));
                this->yolo_data_.last_update_time = this->now();
            }
        }
    );

    if (use_pose_topic_for_current_pose_) {
        current_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            current_pose_topic_, 10,
            [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(pose_mutex_);
                this->pose_data_.x_real = clamp_to_i16(std::round(msg->pose.position.x * 1000.0));
                this->pose_data_.y_real = clamp_to_i16(std::round(msg->pose.position.y * 1000.0));
                this->pose_data_.last_update_time = this->now();
            }
        );
        RCLCPP_INFO(this->get_logger(), "Current pose source: topic '%s'", current_pose_topic_.c_str());
    } else {
        RCLCPP_INFO(this->get_logger(), "Current pose source: TF (%s -> %s)", odom_frame_.c_str(), base_frame_.c_str());
    }
}
