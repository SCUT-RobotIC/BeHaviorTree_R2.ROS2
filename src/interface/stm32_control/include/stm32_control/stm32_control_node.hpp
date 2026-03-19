#ifndef STM32_CONTROL__STM32_CONTROL_NODE_HPP_
#define STM32_CONTROL__STM32_CONTROL_NODE_HPP_

#include "stm32_control/common_headers.hpp"
#include "stm32_control/serial_packet.hpp"
#include "stm32_control/action_codes.hpp"

#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"

#include <memory>
#include <mutex>
#include <string>

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

  struct TargetData {
    int16_t x_target = 0;
    int16_t y_target = 0;
    int16_t heading = 0;
    int16_t stair_direction = 0;
    rclcpp::Time last_update_time{0, 0, RCL_ROS_TIME};
  };

  struct PoseData {
    int16_t x_real = 0;
    int16_t y_real = 0;
    rclcpp::Time last_update_time{0, 0, RCL_ROS_TIME};
  };

  struct YoloData {
    int16_t x_offset = 0;
    int16_t y_offset = 0;
    rclcpp::Time last_update_time{0, 0, RCL_ROS_TIME};
  };

  struct DataSnapshot {
    TargetData target;
    PoseData pose;
    StagData stag;
    ArmData arm;
    YoloData yolo;
  };

  void load_parameters();
  static int16_t clamp_to_i16(double value);
  DataSnapshot build_snapshot();
  void setup_ros_communications();
  void send_timer_callback();
  void fill_pose_fields(serial_protocol::SerialPacket& packet_struct, const DataSnapshot& snapshot);
  void fill_target_fields(serial_protocol::SerialPacket& packet_struct, const DataSnapshot& snapshot);
  void fill_stag_fields(serial_protocol::SerialPacket& packet_struct, const DataSnapshot& snapshot);
  void fill_arm_fields(serial_protocol::SerialPacket& packet_struct, const DataSnapshot& snapshot);
  void fill_yolo_fields(serial_protocol::SerialPacket& packet_struct, const DataSnapshot& snapshot);
  void publish_packet(const serial_protocol::SerialPacket& packet_struct);
  void on_packet_received(const std_msgs::msg::UInt8MultiArray::SharedPtr msg);

  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr tx_pub_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr ack_flag_pub_;
  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr rx_sub_;

  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr target_sub_;
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr action_code_sub_;
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
  std::string action_code_topic_;
  std::string stm32_read_topic_;
  std::string stm32_write_topic_;
  std::string ack_flag_topic_;
  std::string current_pose_topic_;
  bool use_pose_topic_for_current_pose_;
  bool publish_ack_flag_;

  std::atomic<int16_t> latest_action_code_{static_cast<int16_t>(ActionCode::IDLE)};
  std::atomic<bool> action_code_updated_{false};
  std::atomic<int16_t> action_code_to_send_{static_cast<int16_t>(ActionCode::IDLE)};

  TargetData target_data_;
  PoseData pose_data_;
  StagData stag_data_;
  ArmData arm_data_;
  ArmData arm_feedback_;

  bool has_arm_feedback_ = false;

  YoloData yolo_data_;

  std::mutex target_mutex_;
  std::mutex pose_mutex_;
  std::mutex stag_mutex_;
  std::mutex arm_mutex_;
  std::mutex yolo_mutex_;
};

#endif  // STM32_CONTROL__STM32_CONTROL_NODE_HPP_
