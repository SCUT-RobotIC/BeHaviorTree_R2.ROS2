#ifndef COMMON_HEADERS_HPP_
#define COMMON_HEADERS_HPP_

/**
 * @file common_headers.hpp
 * @brief 常用头文件集合，用于stm32_control组件的公共包含项
 *
 * 将 ROS、TF、消息类型以及常用 C++ 标准库头集中在一个文件，
 * 便于在模块内统一包含。
 */

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64_multi_array.hpp" 
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"

// C++ 标准库
#include <vector>
#include <string>
#include <iostream>
#include <thread>
#include <chrono>
#include <cmath>
#include <algorithm>
#include <deque>
#include <mutex>
#include <atomic>

#endif // COMMON_HEADERS_HPP_
