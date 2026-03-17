#ifndef STM32_CONTROL__UTILS_HPP_
#define STM32_CONTROL__UTILS_HPP_

#include "stm32_control/common_headers.hpp"
#include <cstring>

namespace utils {

inline uint16_t rad_to_deg_times10(const double rad) {
  double degrees = rad * 180.0 / M_PI;
  degrees = std::fmod(degrees, 360.0);
  if (degrees < 0) {
    degrees += 360.0;
  }
  return static_cast<uint16_t>(degrees * 10.0);
}

inline uint16_t get_yaw_times10(const geometry_msgs::msg::Quaternion& quat) {
  tf2::Quaternion tf2_quat;
  tf2::fromMsg(quat, tf2_quat);
  double roll, pitch, yaw;
  tf2::Matrix3x3(tf2_quat).getRPY(roll, pitch, yaw);
  return rad_to_deg_times10(yaw);
}

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

template <typename T>
inline T combine_bytes(const uint8_t* data) {
    T result = 0;
    std::memcpy(&result, data, sizeof(T));
    return result;
}

} // namespace utils

#endif // STM32_CONTROL__UTILS_HPP_
