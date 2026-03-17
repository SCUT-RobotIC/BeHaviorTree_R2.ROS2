#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import re
import sys
from rcl_interfaces.msg import SetParametersResult, ParameterDescriptor

class HBSCameraNode(Node):
    def __init__(self):
        super().__init__('hbs_camera_node')
        
        # 初始化CV Bridge
        self.bridge = CvBridge()
        
        # 声明参数 - 图像翻转控制
        flip_param_desc = ParameterDescriptor()
        flip_param_desc.description = "图像翻转控制: 0-完全不翻转, 1-水平垂直翻转"
        
        self.declare_parameter('flip_mode', 0, flip_param_desc)
        
        # 声明帧率参数
        fps_desc = ParameterDescriptor()
        fps_desc.description = "发布帧率 (1-30)"
        self.declare_parameter('publish_fps', 15, fps_desc)

        # 声明设备匹配参数
        device_desc = ParameterDescriptor()
        device_desc.description = "摄像头设备名关键字（在 /dev/v4l/by-id 中匹配），例如 HBS_Camera 或 LRCP_imx415"
        self.declare_parameter('device_name_keyword', 'HBS_Camera', device_desc)
        
        # 设置参数回调
        self.add_on_set_parameters_callback(self.parameters_callback)
        
        # 创建图像发布者
        self.publisher_ = self.create_publisher(Image, 'terraced_camera_image', 10)
        
        # 打开摄像头
        self.cap = self.open_hbs()
        if self.cap is None:
            self.get_logger().error("无法打开HBS摄像头,请尝试lsusb | grep \"HBS Camera\"")
            raise RuntimeError("摄像头初始化失败")
        
        # 获取初始参数值
        self.flip_mode = self.get_parameter('flip_mode').value
        self.publish_fps = self.get_parameter('publish_fps').value
        
        # 创建定时器
        timer_period = 1.0 / self.publish_fps
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info(f"HBS摄像头节点已启动，翻转模式: {self.flip_mode}, 帧率: {self.publish_fps}")
        self.get_logger().info("话题名称: /terraced_camera_image")
    
    def open_hbs(self):
        """打开HBS摄像头"""
        by_id_dir = "/dev/v4l/by-id"
        device_keyword = self.get_parameter('device_name_keyword').value

        if not os.path.isdir(by_id_dir):
            self.get_logger().error(f"{by_id_dir} 不存在")
            return None

        by_id_names = sorted(os.listdir(by_id_dir))
        for name in by_id_names:
            if device_keyword in name and name.endswith("index0"):
                real_path = os.path.realpath(os.path.join(by_id_dir, name))
                m = re.search(r"video(\d+)", real_path)
                if not m:
                    continue
                idx = int(m.group(1))
                cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
                if cap.isOpened():
                    self.get_logger().info(f"HBS摄像头已打开：{name} -> {real_path}")
                    return cap
        self.get_logger().error(
            f"未找到匹配关键字 '{device_keyword}' 的摄像头，或设备无法打开。"
        )
        if by_id_names:
            self.get_logger().error("当前可用 /dev/v4l/by-id 设备: " + ", ".join(by_id_names))
        else:
            self.get_logger().error("/dev/v4l/by-id 目录为空")
        return None
    
    def parameters_callback(self, params):
        """参数变更回调函数"""
        successful = True
        reason = ""
        
        for param in params:
            if param.name == 'flip_mode':
                if param.value in [0, 1]:  # 只允许0或1
                    self.flip_mode = param.value
                    flip_mode_desc = "完全不翻转" if self.flip_mode == 0 else "水平垂直翻转"
                    self.get_logger().info(f"翻转模式已更新: {self.flip_mode} ({flip_mode_desc})")
                else:
                    successful = False
                    reason = f"翻转模式值 {param.value} 无效，只允许0或1"
            
            elif param.name == 'publish_fps':
                if 1 <= param.value <= 30:
                    self.publish_fps = param.value
                    # 重新创建定时器以更新帧率
                    self.timer.cancel()
                    timer_period = 1.0 / self.publish_fps
                    self.timer = self.create_timer(timer_period, self.timer_callback)
                    self.get_logger().info(f"发布帧率已更新: {self.publish_fps}")
                else:
                    successful = False
                    reason = f"帧率值 {param.value} 超出范围 (1-30)"
        
        return SetParametersResult(successful=successful, reason=reason)
    
    def apply_flip(self, frame):
        """应用图像翻转"""
        if self.flip_mode == 0:  # 完全不翻转
            return frame
        elif self.flip_mode == 1:  # 水平垂直翻转
            return cv2.flip(frame, -1)  # -1表示水平和垂直同时翻转
        else:
            # 默认不翻转
            return frame
    
    def timer_callback(self):
        """定时器回调函数 - 发布图像帧"""
        if self.cap is None or not self.cap.isOpened():
            self.get_logger().error("摄像头未就绪")
            return
        
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("无法读取摄像头帧")
            return
        
        # 应用翻转
        flipped_frame = self.apply_flip(frame)
        
        try:
            # 转换为ROS图像消息
            ros_image = self.bridge.cv2_to_imgmsg(flipped_frame, "bgr8")
            ros_image.header.stamp = self.get_clock().now().to_msg()
            ros_image.header.frame_id = "hbs_camera"
            
            # 发布图像到指定话题
            self.publisher_.publish(ros_image)
            
            # 降低日志频率，每5秒记录一次
            self.get_logger().debug(f"发布图像到terraced_camera_image话题，翻转模式: {self.flip_mode}", 
                                  throttle_duration_sec=5.0)
                
        except Exception as e:
            self.get_logger().error(f"图像转换或发布失败: {str(e)}")
    
    def destroy_node(self):
        """节点销毁时清理资源"""
        if self.cap is not None:
            self.cap.release()
            if rclpy.ok():
                self.get_logger().info("摄像头资源已释放")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = None
    
    try:
        node = HBSCameraNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node is not None:
            node.get_logger().info("收到中断信号，节点即将退出")
    except Exception as e:
        error_text = f"{type(e).__name__}: {e}" if str(e) else type(e).__name__
        if node is not None and rclpy.ok():
            node.get_logger().error(f"节点运行错误: {error_text}")
        else:
            print(f"[hbs_camera_node] 节点启动/运行失败: {error_text}", file=sys.stderr)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
