#ifndef COMMON_HEADERS_HPP_
#define COMMON_HEADERS_HPP_

/**
 * @file common_headers.hpp
 * @brief stm32_comm 模块的公共包含头
 *
 * 将 ROS、消息类型与串口库头集中包含，供模块内其他源文件引用。
 * 序列化和协议定义已移至 stm32_protocol_interface 包。
 */

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "serial/serial.h"

// C++ 标准库
#include <cstdint>
#include <exception>
#include <functional>
#include <vector>
#include <string>
#include <iostream>
#include <memory>
#include <thread>
#include <chrono>
#include <algorithm>
#include <deque>
#include <mutex>
#include <atomic>
#include <cstring>
#include <utility>

#endif // COMMON_HEADERS_HPP_
