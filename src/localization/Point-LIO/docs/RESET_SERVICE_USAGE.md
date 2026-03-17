# Point-LIO 重置服务使用说明

## 概述

本功能为 Point-LIO 算法添加了一个 ROS 2 服务接口，允许用户在运行时便捷地重置 Point-LIO 系统，而无需终止整个 launch 进程。

## 功能特性

- **运行时重置**: 无需停止节点即可重置系统状态
- **完整清理**: 清空地图、重置位姿估计和卡尔曼滤波器状态
- **标准接口**: 使用标准 ROS 2 服务 `std_srvs/srv/Trigger`
- **便捷工具**: 提供多种调用方式

## 实现原理

系统利用 Point-LIO 内置的全局重置标志 `flg_reset`，当服务被调用时：

1. 设置 `flg_reset = true`
2. 系统在下一次处理循环中检测到该标志
3. 执行完整的重置操作：
   - 重置 IMU 处理器 (`p_imu->Reset()`)
   - 清空点云数据 (`feats_undistort.reset()`)
   - 重置卡尔曼滤波器状态
   - 重新初始化 ivox 地图
   - 重置首次扫描标志

## 使用方法

### 方法 1: 使用便捷脚本（推荐）

```bash
# 进入脚本目录
cd src/localization/Point-LIO/scripts/

# 执行重置
./reset_point_lio.sh
```

### 方法 2: 使用 ROS 2 命令行

```bash
# 调用重置服务
ros2 service call /point_lio_reset std_srvs/srv/Trigger
```

### 方法 3: 使用测试脚本

```bash
# 运行Python测试脚本
python3 src/localization/Point-LIO/scripts/test_reset_service.py
```

### 方法 4: 在其他 ROS 2 节点中调用

```cpp
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

// 创建服务客户端
auto client = node->create_client<std_srvs::srv::Trigger>("/point_lio_reset");

// 等待服务可用
while (!client->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_INFO(node->get_logger(), "等待Point-LIO重置服务...");
}

// 调用服务
auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
auto future = client->async_send_request(request);

// 处理响应
if (rclcpp::spin_until_future_complete(node, future) == rclcpp::FutureReturnCode::SUCCESS) {
    auto response = future.get();
    if (response->success) {
        RCLCPP_INFO(node->get_logger(), "重置成功: %s", response->message.c_str());
    }
}
```

## 服务信息

- **服务名称**: `/point_lio_reset`
- **服务类型**: `std_srvs/srv/Trigger`
- **请求参数**: 无
- **响应字段**:
  - `success` (bool): 操作是否成功
  - `message` (string): 响应消息

## 注意事项

1. **重置时机**: 重置操作会在下一次处理循环中执行，不是立即生效
2. **数据丢失**: 重置会清空所有累积的地图数据和位姿历史
3. **重新初始化**: 重置后系统需要重新进行初始化过程
4. **服务可用性**: 确保 Point-LIO 节点正在运行且服务可用

## 故障排除

### 1. 服务不可用

```bash
# 检查服务是否存在
ros2 service list | grep point_lio_reset

# 检查 Point-LIO 节点是否运行
ros2 node list | grep laserMapping
```

### 2. 调用失败

- 确保 ROS 2 环境正确设置
- 检查网络连接（如果使用多机器设置）
- 查看 Point-LIO 节点日志输出

### 3. 重置无效果

- 确认服务调用返回成功
- 检查 Point-LIO 节点是否正常处理数据
- 观察重置后的初始化过程

## 构建要求

确保在 `CMakeLists.txt` 中包含以下依赖项：

```cmake
find_package(std_srvs REQUIRED)

ament_target_dependencies(pointlio_mapping
  # ... 其他依赖项 ...
  std_srvs
)
```

## 示例场景

1. **调试过程中**: 当算法出现异常时快速重置
2. **数据收集**: 在不同位置开始新的建图过程
3. **演示展示**: 快速清除之前的地图数据
4. **自动化测试**: 在测试脚本中自动重置系统状态

## 版本历史

- v1.0: 初始实现，添加基本重置功能
- 基于 Point-LIO 原有的 `flg_reset` 机制实现