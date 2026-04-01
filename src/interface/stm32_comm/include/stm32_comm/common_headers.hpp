#ifndef COMMON_HEADERS_HPP_
#define COMMON_HEADERS_HPP_

/**
 * @file common_headers.hpp
 * @brief stm32_comm 模块的公共包含头
 *
 * 将 ROS、消息类型、串口库与常用 C++ 标准库头集中包含，供
 * 模块内其他源文件引用。
 */

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "serial/serial.h"

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
#include <cstring>

#endif // COMMON_HEADERS_HPP_
