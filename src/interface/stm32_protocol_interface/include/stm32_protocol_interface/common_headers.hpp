#ifndef STM32_PROTOCOL_INTERFACE__COMMON_HEADERS_HPP_
#define STM32_PROTOCOL_INTERFACE__COMMON_HEADERS_HPP_

/**
 * @file common_headers.hpp
 * @brief stm32_protocol_interface 的公共头文件
 *
 * 包含序列化、数据包处理和协议定义所需的最少 ROS 和 C++ 标准库依赖。
 */

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_ros/buffer.h"

// C++ 标准库
#include <cstdint>
#include <vector>
#include <string>
#include <iostream>
#include <cmath>
#include <algorithm>
#include <cstring>
#include <chrono>
#include <sstream>
#include <iomanip>

#endif // STM32_PROTOCOL_INTERFACE__COMMON_HEADERS_HPP_
