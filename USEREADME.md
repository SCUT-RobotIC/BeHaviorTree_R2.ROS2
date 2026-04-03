# 如何使用调试
## 在r2_bht目录下source install/setup.bash
```
// 启动定位，上下位机通讯，两个相机相关功能包，行为树
ros2 launch r2_decision bt_excutor.launch.py

// 发布动作码
ros2 topic pub /action_code std_msgs/msg/Byte "{data:[1]"}" --once

// 发布目标点
ros2 topic pub /target_position geometry_msgs/msg/Point "{
  x: 0.5,
  y: -0.5,
  z: 0.0
}" --once
```
