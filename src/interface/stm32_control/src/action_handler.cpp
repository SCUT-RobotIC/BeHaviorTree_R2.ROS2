#include "stm32_control/action_handler.hpp"

#include <limits>

ActionHandler::ActionHandler(
    rclcpp::Logger logger,
    rclcpp::Clock::SharedPtr clock,
    tf2_ros::Buffer& tf_buffer,
    std::string odom_frame_name,
    std::string base_frame_name,
    bool use_pose_topic_for_current_pose,
    std::shared_ptr<Flow> flow  // new parameter
) : logger_(logger),
    clock_(clock),
    tf_buffer_(tf_buffer),
    current_action_(ActionCode::IDLE),
    flow_(std::move(flow)),
    odom_frame_(odom_frame_name),
    base_frame_(base_frame_name),
    use_pose_topic_for_current_pose_(use_pose_topic_for_current_pose)
{
    const auto find_step_idx = [this](const char* step_name, size_t fallback) {
        if (!flow_) {
            return fallback;
        }
        for (size_t i = 0; i < flow_->step_count(); ++i) {
            if (flow_->get_step(i).name == step_name) {
                return i;
            }
        }
        return fallback;
    };

    if (flow_) {
        RCLCPP_INFO(logger_, "ActionHandler initialized with flow '%s'", flow_->name().c_str());
        sequence_step_ = flow_->initial_step();
        if (flow_->auto_activate_on_set()) {
            sequence_active_ = true;
            last_ack_flag_ = 0;
        } else if (flow_->step_count() > 0 && find_step_idx("WaitMove1", flow_->step_count()) < flow_->step_count()) {
            sequence_active_ = true;
            sequence_step_ = find_step_idx("WaitMove1", 0);
            last_ack_flag_ = 0;
        }
    } else {
        RCLCPP_WARN(logger_, "ActionHandler initialized with no flow, defaulting to IDLE");
        sequence_step_ = 0;
    }
}

void ActionHandler::update_current_pose(const geometry_msgs::msg::PoseStamped& pose) {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    latest_pose_ = pose;
}

void ActionHandler::set_flow(std::shared_ptr<Flow> flow) {
    const auto find_step_idx = [&flow](const char* step_name, size_t fallback) {
        if (!flow) {
            return fallback;
        }
        for (size_t i = 0; i < flow->step_count(); ++i) {
            if (flow->get_step(i).name == step_name) {
                return i;
            }
        }
        return fallback;
    };

    flow_ = std::move(flow);
    if (flow_) {
        sequence_step_ = flow_->initial_step();
        sequence_active_ = false;
        if (flow_->auto_activate_on_set()) {
            sequence_active_ = true;
            last_ack_flag_ = 0;
        } else if (flow_->step_count() > 0 && find_step_idx("WaitMove1", flow_->step_count()) < flow_->step_count()) {
            sequence_active_ = true;
            sequence_step_ = find_step_idx("WaitMove1", 0);
            last_ack_flag_ = 0;
        }
        RCLCPP_INFO(logger_, "Flow changed to '%s'", flow_->name().c_str());
    } else {
        sequence_step_ = 0;
        sequence_active_ = false;
        RCLCPP_WARN(logger_, "Flow cleared, using Idle behavior only");
    }
}

void ActionHandler::process_incoming_packet(const serial_protocol::SerialPacket& packet) {
    if (!sequence_active_) {
        last_ack_flag_ = packet.ack_flag;
        return;
    }

    const int16_t ack = packet.ack_flag;
    if (ack == 2 && last_ack_flag_ != 2) {
        advance_on_ack(move2_target_.has_value(), move3_target_.has_value());
    }

    last_ack_flag_ = ack;
}

serial_protocol::SerialPacket ActionHandler::process_outgoing_packet()
{
    serial_protocol::SerialPacket packet_to_send;

    auto log_ctrl_if_changed = [this]() {
        const uint8_t buf_mask =
            (move1_target_.has_value() ? 0b100 : 0) |
            (move2_target_.has_value() ? 0b010 : 0) |
            (move3_target_.has_value() ? 0b001 : 0);

        const bool changed =
            !has_last_ctrl_log_ ||
            sequence_step_ != last_logged_step_ ||
            current_action_ != last_logged_action_ ||
            buf_mask != last_logged_buf_mask_;

        if (!changed) {
            return;
        }
        const char* s_name = get_step_config().name.c_str();

        RCLCPP_INFO(
            logger_,
            "[Ctrl] State: %-10s | Action: %d | Buf: [%d%d%d] | Ack: %d",
            s_name,
            static_cast<int>(current_action_),
            move1_target_.has_value(), move2_target_.has_value(), move3_target_.has_value(),
            last_ack_flag_);

        has_last_ctrl_log_ = true;
        last_logged_step_ = sequence_step_;
        last_logged_action_ = current_action_;
        last_logged_buf_mask_ = buf_mask;
    };

    geometry_msgs::msg::PoseStamped current_pose;
    bool has_pose = false;
    if (use_pose_topic_for_current_pose_) {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        if (latest_pose_.has_value()) {
            current_pose = latest_pose_.value();
            has_pose = true;
        }
    } else {
        current_pose = utils::get_pose(tf_buffer_, logger_, odom_frame_, base_frame_);
        has_pose = true;
    }

    if (!has_pose) {
        RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000, "No pose received from current pose topic yet, using (0, 0)");
        packet_to_send.x_real = 0;
        packet_to_send.y_real = 0;
    } else {
        packet_to_send.x_real = static_cast<int16_t>(current_pose.pose.position.x * 1000.0);
        packet_to_send.y_real = static_cast<int16_t>(current_pose.pose.position.y * 1000.0);
    }
    if (!sequence_active_) {
        current_action_ = ActionCode::IDLE;
        packet_to_send.action = static_cast<int16_t>(static_cast<uint8_t>(current_action_));
        state_idle(packet_to_send);
        log_ctrl_if_changed();
        return packet_to_send;
    }

    const Flow::StepConfig& step_cfg = get_step_config();
    if (step_cfg.finish_sequence) {
        finish_sequence(packet_to_send);
    } else {
        current_action_ = step_cfg.action;
        switch (current_action_) {
            case ActionCode::MOVE: {
                if (step_cfg.move_mode == Flow::MoveMode::RelativeOffset) {
                    const double current_x = has_pose ? current_pose.pose.position.x : 0.0;
                    const double current_y = has_pose ? current_pose.pose.position.y : 0.0;
                    state_move_relative(
                        packet_to_send,
                        current_x,
                        current_y,
                        step_cfg.move_dx_m,
                        step_cfg.move_dy_m);
                } else if (!apply_move_target_from_slot(packet_to_send, step_cfg.move_target_slot)) {
                    state_idle(packet_to_send);
                }
                break;
            }
            case ActionCode::ROTATE: {
                state_idle(packet_to_send);
                const int16_t rotate_dir = step_cfg.rotate_direction.has_value()
                    ? std::clamp<int16_t>(step_cfg.rotate_direction.value(), 0, 3)
                    : target_heading_;
                packet_to_send.target_heading = rotate_dir;
                break;
            }
            case ActionCode::STAIR: {
                state_idle(packet_to_send);
                const int16_t stair_dir = step_cfg.stair_direction.has_value()
                    ? std::clamp<int16_t>(step_cfg.stair_direction.value(), 0, 1)
                    : stair_direction_;
                packet_to_send.stair_direction = stair_dir;
                break;
            }
            default: {
                state_idle(packet_to_send);
                break;
            }
        }
    }

    packet_to_send.action = static_cast<int16_t>(static_cast<uint8_t>(current_action_));
    log_ctrl_if_changed();

    return packet_to_send;
}

ActionCode ActionHandler::get_current_action() const {
    return current_action_;
}

void ActionHandler::set_target_position(double x, double y) {
    const auto now = clock_->now();

    // 短时间重复目标去重（防止发布端突发重复导致缓存被快速填满）
    if (last_accepted_target_.has_value()) {
        const double dx = std::abs(last_accepted_target_->x - x);
        const double dy = std::abs(last_accepted_target_->y - y);
        const double age_sec = (now - last_accepted_target_time_).seconds();
        if (dx <= target_dedup_epsilon_m_ && dy <= target_dedup_epsilon_m_ && age_sec <= target_dedup_window_sec_) {
            RCLCPP_DEBUG(
                logger_,
                "Ignore duplicated target within %.3fs: (%.3f, %.3f)",
                age_sec,
                x,
                y);
            return;
        }
    }

    last_accepted_target_ = TargetPoint{x, y};
    last_accepted_target_time_ = now;

    target_x_ = x;
    target_y_ = y;

    fill_next_target_slot(x, y);
}

// helper wrappers that delegate to the assigned flow
const Flow::StepConfig& ActionHandler::get_step_config() const {
    if (flow_) {
        return flow_->get_step(sequence_step_);
    }
    static Flow::StepConfig kIdle = []() {
        Flow::StepConfig step;
        step.name = "Idle";
        step.action = ActionCode::IDLE;
        step.move_target_slot = TargetSlot::None;
        return step;
    }();
    return kIdle;
}

void ActionHandler::advance_on_ack(bool has_move2, bool has_move3) {
    if (flow_) {
        sequence_step_ = flow_->next_on_ack(sequence_step_, has_move2, has_move3);
    }
}

const std::optional<ActionHandler::TargetPoint>& ActionHandler::get_target_by_slot(TargetSlot slot) const {
    switch (slot) {
        case TargetSlot::Move1:
            return move1_target_;
        case TargetSlot::Move2:
            return move2_target_;
        case TargetSlot::Move3:
            return move3_target_;
        case TargetSlot::None:
        default:
            break;
    }
    static const std::optional<TargetPoint> empty_target = std::nullopt;
    return empty_target;
}

bool ActionHandler::apply_move_target_from_slot(
    serial_protocol::SerialPacket& packet,
    TargetSlot slot)
{
    if (slot == TargetSlot::None) {
        return false;
    }

    const auto& target = get_target_by_slot(slot);
    if (target.has_value()) {
        state_move_target(packet, target->x, target->y);
        return true;
    }

    const auto find_step_idx = [this](const char* step_name, size_t fallback) {
        if (!flow_) {
            return fallback;
        }
        for (size_t i = 0; i < flow_->step_count(); ++i) {
            if (flow_->get_step(i).name == step_name) {
                return i;
            }
        }
        return fallback;
    };

    // If target slot is empty during a MOVE step, move to the matching WAIT step.
    const size_t kIdxMove1 = find_step_idx("Move1", 1);
    const size_t kIdxWaitMove1 = find_step_idx("WaitMove1", kIdxMove1);
    const size_t kIdxMove2 = find_step_idx("Move2", 3);
    const size_t kIdxWaitMove2 = find_step_idx("WaitMove2", 2);
    const size_t kIdxMove3 = find_step_idx("Move3", 7);
    const size_t kIdxWaitMove3 = find_step_idx("WaitMove3", 6);

    if (sequence_step_ == kIdxMove1 && slot == TargetSlot::Move1) {
        sequence_step_ = kIdxWaitMove1;
    } else if (sequence_step_ == kIdxMove2 && slot == TargetSlot::Move2) {
        sequence_step_ = kIdxWaitMove2;
    } else if (sequence_step_ == kIdxMove3 && slot == TargetSlot::Move3) {
        sequence_step_ = kIdxWaitMove3;
    }
    return false;
}

void ActionHandler::finish_sequence(serial_protocol::SerialPacket& packet) {
    // Flow now remains in ended state after completion until new targets/flow changes reactivate it.
    sequence_active_ = false;
    sequence_step_ = 0;

    move1_target_.reset();
    move2_target_.reset();
    move3_target_.reset();
    current_action_ = ActionCode::IDLE;
    state_idle(packet);
}

bool ActionHandler::fill_next_target_slot(double x, double y) {
    const auto find_step_idx = [this](const char* step_name, size_t fallback) {
        if (!flow_) {
            return fallback;
        }
        for (size_t i = 0; i < flow_->step_count(); ++i) {
            if (flow_->get_step(i).name == step_name) {
                return i;
            }
        }
        return fallback;
    };

    const size_t kIdxMove1 = find_step_idx("Move1", 1);
    const size_t kIdxWaitMove1 = find_step_idx("WaitMove1", kIdxMove1);
    const size_t kIdxWaitMove2 = find_step_idx("WaitMove2", 2);
    const size_t kIdxMove2 = find_step_idx("Move2", 3);
    const size_t kIdxWaitMove3 = find_step_idx("WaitMove3", 6);
    const size_t kIdxMove3 = find_step_idx("Move3", 7);

    if (!sequence_active_) {
        if (!move1_target_.has_value()) {
            move1_target_ = TargetPoint{x, y};
            sequence_active_ = true;
            sequence_step_ = kIdxMove1;
            last_ack_flag_ = 0;
            return true;
        }
        return false;
    }

    if (!move2_target_.has_value()) {
        if (sequence_step_ == kIdxWaitMove1 && !move1_target_.has_value()) {
            move1_target_ = TargetPoint{x, y};
            sequence_step_ = kIdxMove1;
            return true;
        }

        move2_target_ = TargetPoint{x, y};
        if (sequence_step_ == kIdxWaitMove2) {
            sequence_step_ = kIdxMove2;
        }
        return true;
    }

    if (!move3_target_.has_value()) {
        move3_target_ = TargetPoint{x, y};
        if (sequence_step_ == kIdxWaitMove3) {
            sequence_step_ = kIdxMove3;
        }
        return true;
    }

    return false;
}

void ActionHandler::set_target_heading(int16_t heading) {
    target_heading_ = std::clamp<int16_t>(heading, 0, 3);
}

void ActionHandler::set_stair_direction(int16_t direction) {
    stair_direction_ = std::clamp<int16_t>(direction, 0, 1);
}

void ActionHandler::state_idle(serial_protocol::SerialPacket& packet) {
    packet.x_target = 0;
    packet.y_target = 0;
    packet.target_heading = target_heading_;
    packet.stair_direction = stair_direction_;
}

void ActionHandler::state_move_target(serial_protocol::SerialPacket& packet, double x, double y) {
    packet.x_target = static_cast<int16_t>(x * 1000.0);
    packet.y_target = static_cast<int16_t>(y * 1000.0);
    packet.target_heading = target_heading_;
    packet.stair_direction = stair_direction_;
}

void ActionHandler::state_move_relative(
    serial_protocol::SerialPacket& packet,
    double current_x,
    double current_y,
    double delta_x,
    double delta_y)
{
    const double target_x = current_x + delta_x;
    const double target_y = current_y + delta_y;
    packet.x_target = static_cast<int16_t>(target_x * 1000.0);
    packet.y_target = static_cast<int16_t>(target_y * 1000.0);
    packet.target_heading = target_heading_;
    packet.stair_direction = stair_direction_;
}
