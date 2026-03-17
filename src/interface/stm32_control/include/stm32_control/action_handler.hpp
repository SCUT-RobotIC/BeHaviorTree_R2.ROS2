#ifndef STM32_CONTROL__ACTION_HANDLER_HPP_
#define STM32_CONTROL__ACTION_HANDLER_HPP_

#include "stm32_control/common_headers.hpp"
#include "stm32_control/action_codes.hpp"
#include "stm32_control/serial_packet.hpp"
#include "stm32_control/utils.hpp"
#include "stm32_control/flow.hpp"    // new flow abstraction

using stm32_control::Flow;  // Flow lives in namespace stm32_control

#include <optional>

class ActionHandler {
public:
    ActionHandler(
        rclcpp::Logger logger,
        rclcpp::Clock::SharedPtr clock,
        tf2_ros::Buffer& tf_buffer,
        std::string odom_frame_name,
        std::string base_frame_name,
        bool use_pose_topic_for_current_pose,
        std::shared_ptr<Flow> flow  // initial flow selection
    );

    ActionHandler(const ActionHandler&) = delete;
    ActionHandler& operator=(const ActionHandler&) = delete;

    void process_incoming_packet(const serial_protocol::SerialPacket& packet);
    serial_protocol::SerialPacket process_outgoing_packet();
    void update_current_pose(const geometry_msgs::msg::PoseStamped& pose);
    
    void set_target_position(double x, double y);
    void set_target_heading(int16_t heading);
    void set_stair_direction(int16_t direction);
    ActionCode get_current_action() const;

    // allow changing the active flow at runtime
    void set_flow(std::shared_ptr<Flow> flow);
    const Flow* current_flow() const { return flow_.get(); }

private:
    void state_idle(serial_protocol::SerialPacket& packet);
    void state_move_target(serial_protocol::SerialPacket& packet, double x, double y);
    void state_move_relative(
        serial_protocol::SerialPacket& packet,
        double current_x,
        double current_y,
        double delta_x,
        double delta_y);

    struct TargetPoint {
        double x = 0.0;
        double y = 0.0;
    };

    using TargetSlot = stm32_control::TargetSlot;  // imported from flow.hpp

    // StepConfig and step/transition logic now live in Flow; ActionHandler only keeps
    // index and pointer to the active flow.

    // sequence index applied when flow_ is not null; 0 usually represents Idle
    // (actual value initialized in constructor)

    // helper wrappers used internally
    const Flow::StepConfig& get_step_config() const;
    void advance_on_ack(bool has_move2, bool has_move3);
    const std::optional<TargetPoint>& get_target_by_slot(TargetSlot slot) const;
    bool apply_move_target_from_slot(serial_protocol::SerialPacket& packet, TargetSlot slot);
    void finish_sequence(serial_protocol::SerialPacket& packet);
    bool fill_next_target_slot(double x, double y);

    rclcpp::Logger logger_;
    rclcpp::Clock::SharedPtr clock_;
    tf2_ros::Buffer& tf_buffer_;
    
    ActionCode current_action_;
    std::shared_ptr<Flow> flow_;               // active flow object
    size_t sequence_step_ = 0;
    bool sequence_active_ = false;
    int16_t last_ack_flag_ = 0;

    std::optional<TargetPoint> move1_target_;
    std::optional<TargetPoint> move2_target_;
    std::optional<TargetPoint> move3_target_;
    std::string odom_frame_;
    std::string base_frame_;
    bool use_pose_topic_for_current_pose_ = false;
    std::optional<geometry_msgs::msg::PoseStamped> latest_pose_;
    std::mutex pose_mutex_;

    // 控制日志去抖：仅在关键字段变化时打印
    bool has_last_ctrl_log_ = false;
    size_t last_logged_step_ = 0;
    ActionCode last_logged_action_ = ActionCode::IDLE;
    uint8_t last_logged_buf_mask_ = 0;

    // 目标去重：忽略短时间内重复到达的同一目标点
    std::optional<TargetPoint> last_accepted_target_;
    rclcpp::Time last_accepted_target_time_{0, 0, RCL_ROS_TIME};
    double target_dedup_window_sec_ = 0.5;
    double target_dedup_epsilon_m_ = 0.005;  // 5mm
    
    double target_x_ = 0.0;
    double target_y_ = 0.0;
    int16_t target_heading_ = 0;   // 0:前, 1:左, 2:后, 3:右
    int16_t stair_direction_ = 0;  // 0:上台阶, 1:下台阶
};

#endif // STM32_CONTROL__ACTION_HANDLER_HPP_
