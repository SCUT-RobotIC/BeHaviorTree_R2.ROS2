#ifndef STM32_CONTROL__ACTION_CODES_HPP_
#define STM32_CONTROL__ACTION_CODES_HPP_

#include <cstdint> 

enum class ActionCode : uint8_t {
  IDLE = 0,             // 待机
  MOVE = 1,             // 目标移动
  GRIP = 2,             // 夹爪夹取
  ALIGN = 3,            // 对齐
  ROTATE = 4,           // 旋转
  STAIR = 5,            // 上下台阶
  ARM_GRIP = 6,         // 机械臂夹取（按轨迹点下发）
  ARM_SUCK = 7,         // 机械臂吸盘动作
  RELEASE_SUCK = 8,     // 机械臂松开吸盘
  STAG_MOVE = 9,        // 矛头与武器杆对齐
  STICK_IN = 10         // 武器杆插入
};

#endif // STM32_CONTROL__ACTION_CODES_HPP_
