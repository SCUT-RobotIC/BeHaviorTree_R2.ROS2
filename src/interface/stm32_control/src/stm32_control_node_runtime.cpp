#include "stm32_control/stm32_control_node.hpp"
#include "stm32_control/serial_packet.hpp"

namespace {

bool is_fixed_arm_debug_flow(const ActionHandler* handler) {
    if (handler == nullptr || handler->current_flow() == nullptr) {
        return false;
    }
    return handler->current_flow()->uses_fixed_arm_debug_trajectory();
}

void load_fixed_arm_debug_trajectory(std::deque<std::array<int16_t, 4>>& queue) {
    queue.clear();

    // Each joint array has length 10, unit: 0.1 degree.
    const std::array<int16_t, 10> joint1 = {100, 120, 140, 160, 180, 160, 140, 120, 100, 80};
    const std::array<int16_t, 10> joint2 = {150, 170, 190, 210, 230, 210, 190, 170, 150, 130};
    const std::array<int16_t, 10> joint3 = {200, 220, 240, 260, 280, 260, 240, 220, 200, 180};
    const std::array<int16_t, 10> yaw = {0, 30, 60, 90, 120, 90, 60, 30, 0, -30};

    for (size_t i = 0; i < joint1.size(); ++i) {
        queue.push_back({joint1[i], joint2[i], joint3[i], yaw[i]});
    }
}

}  // namespace

void Stm32ControlNode::request_arm_angles_from_pose_est_if_needed() {
    if (!use_pose_est_service_for_arm_angles_ || !pose_solve_client_) {
        return;
    }

    {
        std::lock_guard<std::mutex> lock(arm_mutex_);
        if (arm_grip_traj_completed_ || !arm_angle_queue_.empty()) {
            return;
        }
    }

    const auto now = this->now();
    if (pose_est_request_in_flight_) {
        return;
    }

    if (last_pose_est_request_time_.nanoseconds() != 0) {
        const auto elapsed = (now - last_pose_est_request_time_).seconds();
        if (elapsed < pose_est_request_interval_sec_) {
            return;
        }
    }

    if (!pose_solve_client_->service_is_ready()) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "PoseSolve service '%s' is not ready.", pose_est_service_name_.c_str());
        return;
    }

    auto req = std::make_shared<PoseSolve::Request>();
    req->need = true;
    pose_est_request_in_flight_ = true;
    last_pose_est_request_time_ = now;

    pose_solve_client_->async_send_request(
        req,
        [this](rclcpp::Client<PoseSolve>::SharedFuture future) {
            pose_est_request_in_flight_ = false;
            try {
                const auto resp = future.get();
                if (!resp->success) {
                    RCLCPP_WARN(this->get_logger(), "PoseSolve response failed, will retry until success.");
                    return;
                }

                RCLCPP_INFO(this->get_logger(), "接收到机械臂解算结果");

                const size_t n = std::min({
                    resp->theta_sum1.size(),
                    resp->theta_sum2.size(),
                    resp->theta_sum3.size(),
                    resp->theta_sum_yaw.size(),
                    static_cast<size_t>(10)
                });

                if (n == 0) {
                    RCLCPP_WARN(this->get_logger(), "PoseSolve returned empty arm angle arrays.");
                    return;
                }

                std::lock_guard<std::mutex> lock(arm_mutex_);
                arm_angle_queue_.clear();
                arm_grip_traj_completed_ = false;
                arm_traj_total_points_ = n;
                arm_traj_sent_index_ = 0;
                for (size_t i = 0; i < n; ++i) {
                    arm_angle_queue_.push_back({
                        clamp_to_i16(resp->theta_sum1[i] * 10.0),
                        clamp_to_i16(resp->theta_sum2[i] * 10.0),
                        clamp_to_i16(resp->theta_sum3[i] * 10.0),
                        clamp_to_i16(resp->theta_sum_yaw[i] * 10.0)
                    });
                }
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "PoseSolve request failed: %s", e.what());
            }
        }
    );
}

void Stm32ControlNode::send_timer_callback() {
    serial_protocol::SerialPacket packet_struct = action_handler_->process_outgoing_packet();
    const bool is_arm_grip_action =
        packet_struct.action == static_cast<int16_t>(static_cast<uint8_t>(ActionCode::ARM_GRIP));
    const bool use_fixed_arm_debug = is_fixed_arm_debug_flow(action_handler_.get());

    handle_arm_grip_transition(is_arm_grip_action, use_fixed_arm_debug);

    if (is_arm_grip_action && !use_fixed_arm_debug) {
        request_arm_angles_from_pose_est_if_needed();
    }

    fill_stag_fields(packet_struct);
    fill_arm_fields(packet_struct, is_arm_grip_action, use_fixed_arm_debug);
    fill_yolo_fields(packet_struct);
    publish_packet(packet_struct);
}

void Stm32ControlNode::handle_arm_grip_transition(bool is_arm_grip_action, bool use_fixed_arm_debug) {

    {
        std::lock_guard<std::mutex> lock(arm_mutex_);
        if (is_arm_grip_action && !last_cycle_arm_grip_action_) {
            arm_angle_queue_.clear();
            arm_grip_traj_completed_ = false;
            arm_traj_total_points_ = 0;
            arm_traj_sent_index_ = 0;
            pose_est_request_in_flight_ = false;
            // Reset request timestamp so we send the first PoseSolve request
            // immediately after entering ArmGrip state.
            last_pose_est_request_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

            if (use_fixed_arm_debug) {
                load_fixed_arm_debug_trajectory(arm_angle_queue_);
                arm_traj_total_points_ = arm_angle_queue_.size();
                arm_traj_sent_index_ = 0;
                arm_grip_traj_completed_ = false;
                RCLCPP_INFO(this->get_logger(),
                    "Arm fixed debug trajectory loaded: %zu points.",
                    arm_traj_total_points_);
            }
        } else if (!is_arm_grip_action && last_cycle_arm_grip_action_) {
            arm_angle_queue_.clear();
            arm_grip_traj_completed_ = false;
            arm_traj_total_points_ = 0;
            arm_traj_sent_index_ = 0;
        }
        last_cycle_arm_grip_action_ = is_arm_grip_action;
    }

}

void Stm32ControlNode::fill_stag_fields(serial_protocol::SerialPacket& packet_struct) {
    {
        std::lock_guard<std::mutex> lock(stag_mutex_);
        auto now = this->now();
        auto age = (now - stag_data_.last_update_time).seconds();
        if (stag_data_.detected && age < 0.5) {
            packet_struct.stag_x = stag_data_.x;
            packet_struct.stag_y = stag_data_.y;
            packet_struct.stag_detected = 1;
        } else {
            packet_struct.stag_x = 0;
            packet_struct.stag_y = 0;
            packet_struct.stag_detected = 0;
        }
    }
}

void Stm32ControlNode::fill_arm_fields(serial_protocol::SerialPacket& packet_struct, bool is_arm_grip_action, bool use_fixed_arm_debug) {
    {
        std::lock_guard<std::mutex> lock(arm_mutex_);
        auto now = this->now();
        packet_struct.arm_send_count = 0;

        if (is_arm_grip_action) {
            if (!arm_angle_queue_.empty()) {
                const auto target = arm_angle_queue_.front();
                packet_struct.arm_send_count = static_cast<int16_t>(std::clamp<size_t>(arm_traj_sent_index_ + 1, 1, 10));

                packet_struct.arm_joint1 = target[0];
                packet_struct.arm_joint2 = target[1];
                packet_struct.arm_joint3 = target[2];
                packet_struct.arm_yaw = target[3];
                
                arm_data_.joint1 = target[0];
                arm_data_.joint2 = target[1];
                arm_data_.joint3 = target[2];
                arm_data_.yaw = target[3];
                arm_data_.last_update_time = now;

                arm_angle_queue_.pop_front();
                ++arm_traj_sent_index_;
                if (arm_angle_queue_.empty()) {
                    if (use_fixed_arm_debug) {
                        load_fixed_arm_debug_trajectory(arm_angle_queue_);
                        arm_traj_total_points_ = arm_angle_queue_.size();
                        arm_traj_sent_index_ = 0;
                        arm_grip_traj_completed_ = false;
                    } else {
                        arm_grip_traj_completed_ = true;
                        RCLCPP_INFO(this->get_logger(), "Arm grip trajectory points all sent and matched.");
                    }
                }
                
            } else if (arm_grip_traj_completed_) {
                packet_struct.arm_send_count = static_cast<int16_t>(std::clamp<size_t>(arm_traj_total_points_, 0, 10));
                packet_struct.arm_joint1 = arm_data_.joint1;
                packet_struct.arm_joint2 = arm_data_.joint2;
                packet_struct.arm_joint3 = arm_data_.joint3;
                packet_struct.arm_yaw = arm_data_.yaw;
            } else {
                packet_struct.arm_send_count = 0;
                packet_struct.arm_joint1 = 0;
                packet_struct.arm_joint2 = 0;
                packet_struct.arm_joint3 = 0;
                packet_struct.arm_yaw = 0;
            }
        } else {
            packet_struct.arm_send_count = 0;
            packet_struct.arm_joint1 = 0;
            packet_struct.arm_joint2 = 0;
            packet_struct.arm_joint3 = 0;
            packet_struct.arm_yaw = 0;
        }
    }
}

void Stm32ControlNode::fill_yolo_fields(serial_protocol::SerialPacket& packet_struct) {
    {
        std::lock_guard<std::mutex> lock(yolo_mutex_);
        auto now = this->now();
        auto age = (now - yolo_data_.last_update_time).seconds();
        if (age < 0.5 && yolo_data_.last_update_time.nanoseconds() != 0) {
            packet_struct.x_offset = yolo_data_.x_offset;
            packet_struct.y_offset = yolo_data_.y_offset;
        } else {
            packet_struct.x_offset = 0;
            packet_struct.y_offset = 0;
        }
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

        {
            std::lock_guard<std::mutex> lock(arm_mutex_);
            arm_feedback_.joint1 = packet_struct.arm_joint1;
            arm_feedback_.joint2 = packet_struct.arm_joint2;
            arm_feedback_.joint3 = packet_struct.arm_joint3;
            arm_feedback_.yaw = packet_struct.arm_yaw;
            arm_feedback_.last_update_time = this->now();
            has_arm_feedback_ = true;
        }

        action_handler_->process_incoming_packet(packet_struct);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to process received packet: %s", e.what());
    }
}
