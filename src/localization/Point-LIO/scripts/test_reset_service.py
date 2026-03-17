#!/usr/bin/env python3
"""
Point-LIO重置服务测试脚本
用于测试Point-LIO的重置功能
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import sys


class PointLIOResetClient(Node):
    
    def __init__(self):
        super().__init__('point_lio_reset_client')
        self.client = self.create_client(Trigger, '/point_lio_reset')
        
        # 等待服务可用
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待Point-LIO重置服务可用...')
    
    def call_reset_service(self):
        """调用重置服务"""
        request = Trigger.Request()
        
        self.get_logger().info('正在调用Point-LIO重置服务...')
        
        try:
            future = self.client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            
            if future.result() is not None:
                response = future.result()
                if response.success:
                    self.get_logger().info(f'重置服务调用成功: {response.message}')
                    return True
                else:
                    self.get_logger().error(f'重置服务调用失败: {response.message}')
                    return False
            else:
                self.get_logger().error('服务调用失败：未收到响应')
                return False
                
        except Exception as e:
            self.get_logger().error(f'服务调用异常: {str(e)}')
            return False


def main():
    rclpy.init()
    
    reset_client = PointLIOResetClient()
    
    try:
        success = reset_client.call_reset_service()
        if success:
            print("✅ Point-LIO重置服务测试成功")
            sys.exit(0)
        else:
            print("❌ Point-LIO重置服务测试失败")
            sys.exit(1)
            
    except KeyboardInterrupt:
        print("\n测试被用户中断")
        
    finally:
        reset_client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()