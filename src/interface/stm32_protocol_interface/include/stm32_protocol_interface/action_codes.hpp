#ifndef STM32_PROTOCOL_INTERFACE__ACTION_CODES_HPP_
#define STM32_PROTOCOL_INTERFACE__ACTION_CODES_HPP_

#include <cstdint>

/**
 * @file action_codes.hpp
 * @brief 定义 STM32 控制协议中使用的动作码枚举
 */

/**
 * @enum ActionCode
 * @brief 表示发送给 STM32 的动作命令码
 */
enum class ActionCode : uint8_t {
  IDLE = 0,             ///< 待机
  MOVE = 1,             ///< 目标移动
  GRIP = 2,             ///< 夹爪夹取
  ALIGN = 3,            ///< 对齐
  ROTATE = 4,           ///< 旋转
  STAIR = 5,            ///< 上下台阶
  ARM_GRIP = 6,         ///< 机械臂夹取（按轨迹点下发）
  ARM_SUCK = 7,         ///< 机械臂吸盘动作
  RELEASE_SUCK = 8,     ///< 机械臂松开吸盘
  STAG_MOVE = 9,        ///< 矛头与武器杆对齐
  STICK_IN = 10,         ///< 武器杆插入

  // --- Stair/special actions (0x20 - 0x2F) --------------------------------
  SUPPORTS_LIFT_ALL = 32,      // 所有支撑向下撑起车体
  SUPPORTS_RETRACT_FRONT = 33, // 收起前侧支撑
  MOVE_DUAL_WHEEL_SYNC = 34,   // 大轮小轮同步走定点
  SUPPORTS_RETRACT_REAR = 35   // 收起后侧支撑
};

#endif // STM32_PROTOCOL_INTERFACE__ACTION_CODES_HPP_
