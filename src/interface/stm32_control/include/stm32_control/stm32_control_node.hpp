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

/**
 * @file stm32_control_node.hpp
 * @brief STM32 控制节点类声明，负责构建控制数据包并与串口通信节点交互
 */

/**
 * @class Stm32ControlNode
 * @brief ROS2 节点：根据传感与目标数据定时构建并发布给串口通信节点
 *
 * 节点从多个主题订阅输入（目标点、姿态、机械臂角度、YOLO 偏移等），
 * 按设定频率构建 `serial_protocol::SerialPacket` 并通过 `tx_pub_` 发布给
 * 串口通信层（`stm32_comm`）。同时接收来自串口层的反馈并更新内部状态。
 */
class Stm32ControlNode : public rclcpp::Node {
public:
  Stm32ControlNode();

private:
  // stag码数据结构，包含位置、检测状态和最后更新时间
  struct StagData {
    int16_t x = 0;
    int16_t y = 0;
    bool detected = false;
    rclcpp::Time last_update_time{0, 0, RCL_ROS_TIME};
  };

  // 机械臂数据结构，包含关节角度、底盘偏航角和最后更新时间
  struct ArmData {
    int16_t arm_send_count = 0; // 机械臂数据发送计数，0表示无效数据，1表示有效数据
    int16_t joint1 = 0;
    int16_t joint2 = 0;
    int16_t joint3 = 0;
    int16_t yaw = 0;
    rclcpp::Time last_update_time{0, 0, RCL_ROS_TIME};
  };

  // 目标数据结构，包含目标位置、朝向和最后更新时间
  struct TargetData {
    int16_t x_target = 0;
    int16_t y_target = 0;
    int16_t target_heading = 0;
    int16_t stair_direction = 0;
    int16_t stair_lift_height = 0;
    rclcpp::Time last_update_time{0, 0, RCL_ROS_TIME};
  };

  // 位置结构，包含当前x、y坐标和最后更新时间
  struct PoseData {
    int16_t x_real = 0;
    int16_t y_real = 0;
    rclcpp::Time last_update_time{0, 0, RCL_ROS_TIME};
  };

  // 夹取矛头YOLO偏移数据结构，包含x、y偏移和最后更新时间
  struct YoloData {
    int16_t x_offset = 0;
    int16_t y_offset = 0;
    rclcpp::Time last_update_time{0, 0, RCL_ROS_TIME};
  };

  // 数据快照结构，包含所有输入数据的当前值，用于构建发送数据包
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

  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr tx_pub_;                 // 发布给串口通信层的主题
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr ack_flag_pub_;                     // 可选的 ACK 标志发布者
  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr rx_sub_;              // 订阅来自串口通信层的反馈数据

  rclcpp::Subscription<geometry_msgs::msg::Int16>::SharedPtr stair_lift_height_sub_;               // 目标点订阅
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr target_sub_;               // 目标点订阅
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr target_heading_sub_;            // 目标朝向订阅
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr action_code_sub_;               // 动作码订阅
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr stair_direction_sub_;           // 台阶方向订阅
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr stag_center_sub_;          // STAG码中心点订阅
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stag_detected_sub_;              // STAG码检测状态订阅
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr arm_angles_sub_;    // 机械臂角度订阅
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr yolo_offsets_sub_;  // 矛头夹取YOLO偏移订阅
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_sub_;   // 当前位姿订阅（可选）

  rclcpp::TimerBase::SharedPtr send_timer_;                                             // 定时器，用于周期性发送控制数据包 

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;                                          // TF2 缓冲区，用于坐标变换 
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;                             // TF2 监听器，用于接收坐标变换信息 

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
  YoloData yolo_data_;

  bool has_arm_feedback_ = false;

  std::mutex target_mutex_;
  std::mutex pose_mutex_;
  std::mutex stag_mutex_;
  std::mutex arm_mutex_;
  std::mutex yolo_mutex_;
};

#endif  // STM32_CONTROL__STM32_CONTROL_NODE_HPP_
