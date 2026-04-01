#ifndef STM32_CONTROL__UTILS_HPP_
#define STM32_CONTROL__UTILS_HPP_

#include "stm32_control/common_headers.hpp"
#include <cstring>

/**
 * @file utils.hpp
 * @brief 工具函数集合：角度转换、TF 获取以及字节合并等小工具
 */

namespace utils {

/**
 * @brief 将弧度转换为以 0.1 度为单位的无符号整数
 * @param rad 弧度值
 * @return 返回 degrees*10 的 uint16_t 表示（范围 [0, 3600)）
 */
inline uint16_t rad_to_deg_times10(const double rad) {
  double degrees = rad * 180.0 / M_PI;
  degrees = std::fmod(degrees, 360.0);
  if (degrees < 0) {
    degrees += 360.0;
  }
  return static_cast<uint16_t>(degrees * 10.0);
}

/**
 * @brief 从四元数提取 yaw（以 0.1 度为单位）
 * @param quat 输入四元数
 * @return yaw*10 的 uint16_t 表示
 */
inline uint16_t get_yaw_times10(const geometry_msgs::msg::Quaternion& quat) {
  tf2::Quaternion tf2_quat;
  tf2::fromMsg(quat, tf2_quat);
  double roll, pitch, yaw;
  tf2::Matrix3x3(tf2_quat).getRPY(roll, pitch, yaw);
  return rad_to_deg_times10(yaw);
}

/**
 * @brief 使用 TF 缓冲区查询两个坐标系之间的 pose
 * @param tf_buffer TF 缓冲区
 * @param logger 日志记录器（用于警告输出）
 * @param target_frame 目标坐标系
 * @param source_frame 源坐标系
 * @return 若查询成功则填充 pose 的 header 与 pose 字段，否则返回空 pose
 */
inline geometry_msgs::msg::PoseStamped get_pose(
    tf2_ros::Buffer& tf_buffer, 
    const rclcpp::Logger& logger, 
    const std::string& target_frame, 
    const std::string& source_frame) 
{
  geometry_msgs::msg::PoseStamped pose;
  try {
    geometry_msgs::msg::TransformStamped transform = tf_buffer.lookupTransform(
        target_frame, source_frame, tf2::TimePointZero, std::chrono::seconds(1));

    pose.header.stamp = transform.header.stamp;
    pose.header.frame_id = target_frame;
    pose.pose.position.x = transform.transform.translation.x;
    pose.pose.position.y = transform.transform.translation.y;
    pose.pose.position.z = transform.transform.translation.z;
    pose.pose.orientation = transform.transform.rotation;
  } catch (const tf2::TransformException& ex) {
    RCLCPP_WARN(logger, "Could not get transform from '%s' to '%s': %s", 
                source_frame.c_str(), target_frame.c_str(), ex.what());
  }
  return pose;
}

/**
 * @brief 将连续字节合并为指定类型（按内存对齐复制）
 * @tparam T 目标整型类型
 * @param data 指向数据的指针（小端/字节顺序由上层保证）
 * @return 合并后的值
 */
template <typename T>
inline T combine_bytes(const uint8_t* data) {
    T result = 0;
    std::memcpy(&result, data, sizeof(T));
    return result;
}

} // namespace utils

#endif // STM32_CONTROL__UTILS_HPP_
