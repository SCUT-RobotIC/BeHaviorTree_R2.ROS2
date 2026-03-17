#ifndef STM32_COMM__ACTION_CODES_HPP_
#define STM32_COMM__ACTION_CODES_HPP_

#include <cstdint> // For uint8_t

/**
 * @file action_codes.hpp
 * @description Copied from stm32_ros_interface_cpp to support compilation of SerialPacket.
 */

enum class ActionCode : uint8_t {
  IDLE = 0,             // 机器人空闲/待机状态
};

#endif // STM32_COMM__ACTION_CODES_HPP_
