 #!/bin/bash

# Point-LIO重置便捷脚本
# 使用方法: ./reset_point_lio.sh

echo "🔄 正在调用Point-LIO重置服务..."

# 检查ROS 2环境是否设置
if ! command -v ros2 &> /dev/null; then
    echo "❌ 错误: 未找到ros2命令，请确保ROS 2环境已正确设置"
    exit 1
fi

# 使用ros2 service call命令调用重置服务
if ros2 service call /point_lio_reset std_srvs/srv/Trigger; then
    echo "✅ Point-LIO重置命令已发送成功"
    echo "📝 系统将在下一次处理循环中执行重置操作"
else
    echo "❌ 重置服务调用失败"
    echo "🔍 请检查:"
    echo "   1. Point-LIO节点是否正在运行"
    echo "   2. 重置服务是否可用: ros2 service list | grep point_lio_reset"
    exit 1
fi