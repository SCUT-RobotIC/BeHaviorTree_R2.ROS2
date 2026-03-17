#ifndef STM32_CONTROL__STM32_CONTROL_NODE_HPP_
#define STM32_CONTROL__STM32_CONTROL_NODE_HPP_

#include "stm32_control/common_headers.hpp"
#include "stm32_control/action_handler.hpp"

#include "foundationpose_interface/srv/pose_solve.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"

#include <array>
#include <deque>
#include <memory>
#include <mutex>
#include <string>

using PoseSolve = foundationpose_interface::srv::PoseSolve;

namespace serial_protocol {
struct SerialPacket;
}

class Stm32ControlNode : public rclcpp::Node {
public:
  Stm32ControlNode();

private:
  struct StagData {
    int16_t x = 0;
    int16_t y = 0;
    bool detected = false;
    rclcpp::Time last_update_time{0, 0, RCL_ROS_TIME};
  };

  struct ArmData {
    int16_t joint1 = 0;
    int16_t joint2 = 0;
    int16_t joint3 = 0;
    int16_t yaw = 0;
    rclcpp::Time last_update_time{0, 0, RCL_ROS_TIME};
  };

  struct YoloData {
    int16_t x_offset = 0;
    int16_t y_offset = 0;
    rclcpp::Time last_update_time{0, 0, RCL_ROS_TIME};
  };

  void load_parameters();
  static int16_t clamp_to_i16(double value);
  void request_arm_angles_from_pose_est_if_needed();
  void setup_ros_communications();
  void send_timer_callback();
  void handle_arm_grip_transition(bool is_arm_grip_action, bool use_fixed_arm_debug);
  void fill_stag_fields(serial_protocol::SerialPacket& packet_struct);
  void fill_arm_fields(serial_protocol::SerialPacket& packet_struct, bool is_arm_grip_action, bool use_fixed_arm_debug);
  void fill_yolo_fields(serial_protocol::SerialPacket& packet_struct);
  void publish_packet(const serial_protocol::SerialPacket& packet_struct);
  void on_packet_received(const std_msgs::msg::UInt8MultiArray::SharedPtr msg);

  std::unique_ptr<ActionHandler> action_handler_;

  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr tx_pub_;
  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr rx_sub_;

  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr target_sub_;
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr target_heading_sub_;
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr stair_direction_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr stag_center_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stag_detected_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr arm_angles_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr yolo_offsets_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_sub_;

  rclcpp::TimerBase::SharedPtr send_timer_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  double frequency_ = 100.0;
  std::string odom_frame_;
  std::string base_frame_;
  std::string goal_pose_topic_;
  std::string stag_center_topic_;
  std::string stag_detected_topic_;
  std::string target_heading_topic_;
  std::string stair_direction_topic_;
  std::string arm_angles_topic_;
  std::string yolo_offsets_topic_;
  std::string stm32_read_topic_;
  std::string stm32_write_topic_;
  std::string current_pose_topic_;
  std::string pose_est_service_name_;
  std::string flow_name_;
  bool use_pose_topic_for_current_pose_;
  bool use_pose_est_service_for_arm_angles_;
  double pose_est_request_interval_sec_;

  rclcpp::Client<PoseSolve>::SharedPtr pose_solve_client_;
  bool pose_est_request_in_flight_ = false;
  rclcpp::Time last_pose_est_request_time_{0, 0, RCL_ROS_TIME};

  StagData stag_data_;
  ArmData arm_data_;
  ArmData arm_feedback_;

  bool has_arm_feedback_ = false;
  bool arm_grip_traj_completed_ = false;
  bool last_cycle_arm_grip_action_ = false;
  std::deque<std::array<int16_t, 4>> arm_angle_queue_;
  size_t arm_traj_total_points_ = 0;
  size_t arm_traj_sent_index_ = 0;

  YoloData yolo_data_;

  std::mutex stag_mutex_;
  std::mutex arm_mutex_;
  std::mutex yolo_mutex_;
};

#endif  // STM32_CONTROL__STM32_CONTROL_NODE_HPP_
