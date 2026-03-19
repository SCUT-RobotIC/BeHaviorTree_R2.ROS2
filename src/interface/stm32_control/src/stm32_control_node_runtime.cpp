#include "stm32_control/stm32_control_node.hpp"
#include "stm32_control/serial_packet.hpp"

Stm32ControlNode::DataSnapshot Stm32ControlNode::build_snapshot() {
    DataSnapshot snapshot;

    {
        std::lock_guard<std::mutex> lock(target_mutex_);
        snapshot.target = target_data_;
    }

    {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        snapshot.pose = pose_data_;
    }

    {
        std::lock_guard<std::mutex> lock(stag_mutex_);
        snapshot.stag = stag_data_;
    }

    {
        std::lock_guard<std::mutex> lock(arm_mutex_);
        snapshot.arm = arm_data_;
    }

    {
        std::lock_guard<std::mutex> lock(yolo_mutex_);
        snapshot.yolo = yolo_data_;
    }

    return snapshot;
}

void Stm32ControlNode::send_timer_callback() {
    const DataSnapshot snapshot = build_snapshot();

    serial_protocol::SerialPacket packet_struct;
    if (action_code_updated_.exchange(false, std::memory_order_acq_rel)) {
        action_code_to_send_.store(
            latest_action_code_.load(std::memory_order_relaxed),
            std::memory_order_relaxed);
    }
    packet_struct.action = action_code_to_send_.load(std::memory_order_relaxed);

    fill_pose_fields(packet_struct, snapshot);
    fill_target_fields(packet_struct, snapshot);
    fill_stag_fields(packet_struct, snapshot);
    fill_arm_fields(packet_struct, snapshot);
    fill_yolo_fields(packet_struct, snapshot);
    publish_packet(packet_struct);
}

void Stm32ControlNode::fill_pose_fields(
    serial_protocol::SerialPacket& packet_struct,
    const DataSnapshot& snapshot) {
    if (!use_pose_topic_for_current_pose_) {
        try {
            const auto transform = tf_buffer_->lookupTransform(
                odom_frame_, base_frame_, tf2::TimePointZero);
            packet_struct.x_real = clamp_to_i16(std::round(transform.transform.translation.x * 1000.0));
            packet_struct.y_real = clamp_to_i16(std::round(transform.transform.translation.y * 1000.0));
            return;
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "TF lookup failed (%s -> %s): %s",
                odom_frame_.c_str(), base_frame_.c_str(), ex.what());
        }
    }

    packet_struct.x_real = snapshot.pose.x_real;
    packet_struct.y_real = snapshot.pose.y_real;
}

void Stm32ControlNode::fill_target_fields(
    serial_protocol::SerialPacket& packet_struct,
    const DataSnapshot& snapshot) {
    packet_struct.x_target = snapshot.target.x_target;
    packet_struct.y_target = snapshot.target.y_target;
    packet_struct.target_heading = snapshot.target.heading;
    packet_struct.stair_direction = snapshot.target.stair_direction;
}

void Stm32ControlNode::fill_stag_fields(
    serial_protocol::SerialPacket& packet_struct,
    const DataSnapshot& snapshot) {
    auto now = this->now();
    auto age = (now - snapshot.stag.last_update_time).seconds();
    if (snapshot.stag.detected && age < 0.5) {
        packet_struct.stag_x = snapshot.stag.x;
        packet_struct.stag_y = snapshot.stag.y;
        packet_struct.stag_detected = 1;
    } else {
        packet_struct.stag_x = 0;
        packet_struct.stag_y = 0;
        packet_struct.stag_detected = 0;
    }
}

void Stm32ControlNode::fill_arm_fields(
    serial_protocol::SerialPacket& packet_struct,
    const DataSnapshot& snapshot) {
    auto now = this->now();
    auto age = (now - snapshot.arm.last_update_time).seconds();
    if (snapshot.arm.last_update_time.nanoseconds() == 0 || age >= 0.5) {
        packet_struct.arm_send_count = 0;
        packet_struct.arm_joint1 = 0;
        packet_struct.arm_joint2 = 0;
        packet_struct.arm_joint3 = 0;
        packet_struct.arm_yaw = 0;
    } else {
        packet_struct.arm_send_count = 1;
        packet_struct.arm_joint1 = snapshot.arm.joint1;
        packet_struct.arm_joint2 = snapshot.arm.joint2;
        packet_struct.arm_joint3 = snapshot.arm.joint3;
        packet_struct.arm_yaw = snapshot.arm.yaw;
    }
}

void Stm32ControlNode::fill_yolo_fields(
    serial_protocol::SerialPacket& packet_struct,
    const DataSnapshot& snapshot) {
    auto now = this->now();
    auto age = (now - snapshot.yolo.last_update_time).seconds();
    if (age < 0.5 && snapshot.yolo.last_update_time.nanoseconds() != 0) {
        packet_struct.x_offset = snapshot.yolo.x_offset;
        packet_struct.y_offset = snapshot.yolo.y_offset;
    } else {
        packet_struct.x_offset = 0;
        packet_struct.y_offset = 0;
    }
}

void Stm32ControlNode::publish_packet(const serial_protocol::SerialPacket& packet_struct) {
    std::vector<uint8_t> buffer_to_send;
    serial_protocol::serialize_packet(packet_struct, buffer_to_send);

    std_msgs::msg::UInt8MultiArray msg;
    msg.data = buffer_to_send;
    tx_pub_->publish(msg);
}

void Stm32ControlNode::on_packet_received(const std_msgs::msg::UInt8MultiArray::SharedPtr msg) {
    try {
        serial_protocol::SerialPacket packet_struct = serial_protocol::deserialize_packet(msg->data);

        if (publish_ack_flag_ && ack_flag_pub_) {
            std_msgs::msg::Int16 ack_msg;
            ack_msg.data = packet_struct.ack_flag;
            ack_flag_pub_->publish(ack_msg);
        }

        {
            std::lock_guard<std::mutex> lock(arm_mutex_);
            arm_feedback_.joint1 = packet_struct.arm_joint1;
            arm_feedback_.joint2 = packet_struct.arm_joint2;
            arm_feedback_.joint3 = packet_struct.arm_joint3;
            arm_feedback_.yaw = packet_struct.arm_yaw;
            arm_feedback_.last_update_time = this->now();
            has_arm_feedback_ = true;
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to process received packet: %s", e.what());
    }
}
