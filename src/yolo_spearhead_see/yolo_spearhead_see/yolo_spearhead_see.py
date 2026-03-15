#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import threading
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, Int16
import cv2
from collections import deque
from cv_bridge import CvBridge
from ultralytics import YOLO
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
import rclpy.action
from fetchsp_interfaces.action import FetchSp

class YOLOSpearheadSeeNode(Node):
    def __init__(self):
        super().__init__('yolo_spearhead_see')
        self.declare_parameter('debug_level', 0)  # 调试级别参数，默认为0（无调试输出）
        self.declare_parameter('error', 20.0)  # 偏差误差范围，单位为像素
        self.declare_parameter('threshold', 0.8)  # 置信度阈值
        self.declare_parameter('window_size',5)
        self.declare_parameter('max_miss_frames',1)
        
        self.error = self.get_parameter('error').get_parameter_value().double_value  # 获取偏差误差范围
        self.threshold = self.get_parameter('threshold').get_parameter_value().double_value  # 获取置信度阈值
        self.debug_level = self.get_parameter('debug_level').get_parameter_value().integer_value  # 获取调试级别参数值
        self.window_size = self.get_parameter('window_size').get_parameter_value().integer_value  # 获取窗口大小参数值
        self.max_miss_frames = self.get_parameter('max_miss_frames').get_parameter_value().integer_value  # 获取最大丢失帧数参数值

        # 加载YOLO模型
        self.model = YOLO('src/perception/yolo_spearhead_see/models/blackwhite_totalspearhead_yolov8s.pt')
        
        # 初始化CV桥接器
        self.bridge = CvBridge()
        
        self.data_lock = threading.Lock()
        self.capture_event = threading.Event()  # 用于控制图像捕获的事件
        self.ACK = 0  # 初始化ACK状态
        self.cv_image = None  # 存储当前图像帧
        self.offset_window = deque(maxlen=self.window_size)  # 用于存储偏差值的滑动窗口
        self.last_valid_offsets = None  # 存储上一次有效的偏差值
        self.miss_count = 0  # 计数连续丢失帧数

        # 创建回调组
        self.action_group = ReentrantCallbackGroup()
        self.sub_group = ReentrantCallbackGroup()

        self.FetchSp_Action_Server = rclpy.action.ActionServer(
            self,
            FetchSp,
            'camera_stag_action',
            execute_callback = self.execute_callback,
            goal_callback = self.goal_callback,
            cancel_callback = self.cancel_callback,
            callback_group=self.action_group
        )

        # 订阅相机话题
        self.subscription = self.create_subscription(
            Image,
            '/terraced_camera_image',
            self.image_callback,
            10,
            callback_group=self.sub_group
        )

        self.ACK_sub = self.create_subscription(
            Int16, 
            'ACK',
            self.ACK_callback,
            10,
            callback_group=self.sub_group
        )
        
        # 发布识别结果的偏差数组
        self.offset_publisher = self.create_publisher(
            Float32MultiArray, 
            '/yolo_detection_offsets', 
            10)
        
        # 发布标注后的图像
        self.annotated_image_publisher = self.create_publisher(
            Image,
            '/yolo_annotated_image',
            10)
        
        self.get_logger().info('YOLO Spearhead See节点已启动，等待图像输入...')
    
    def ACK_callback(self, msg):
        with self.data_lock:
            self.ACK = msg.data
        if self.ACK == 2:
            self.capture_event.set()  # 触发捕获事件
        else:
            self.capture_event.clear()  # 清除捕获事件

    def image_callback(self, msg):
        try:
            # 将ROS图像消息转换为OpenCV格式
            with self.data_lock:
                self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'图像转换错误: {e}')
            return

    def execute_callback(self, goal_handle):    
        self.get_logger().info('执行相机检测任务...')
        if self.cv_image is None:
            self.get_logger().warning('尚未接收到图像，无法执行检测')
            return FetchSp.Result(success=False)

        while not self.capture_event.wait(timeout=0.1):  # 等待捕获事件被触发
            
            # 使用YOLO模型进行推理
            with self.data_lock:
                frame = self.cv_image.copy()  # 复制当前图像帧以避免线程冲突
            if frame is None:
                self.miss_count += 1
                continue
            # 获取图像中心坐标
            height, width = frame.shape[:2]
            center_x, center_y = width // 2, height // 2

            results = self.model(frame)

            detections = []
            # 遍历检测结果
            for result in results:
                # 获取边界框坐标
                boxes = result.boxes

                if boxes is None or len(boxes) == 0:
                    continue

                # 提取所有检测框的中心坐标和置信度，并存储在列表中
                for box in boxes:
                    if box.conf.item() < self.threshold:  # 过滤掉置信度较低的检测结果
                        continue
                    # 获取边界框坐标 (x1, y1, x2, y2)
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    
                    # 计算边界框中心点
                    box_center_x = (x1 + x2) / 2
                    box_center_y = (y1 + y2) / 2
                          
                    # 计算相对于图像中心的像素偏差（不归一化）
                    x_offset = box_center_x - center_x  # 正值为右偏，负值为左偏
                    y_offset = box_center_y - center_y  # 正值为下偏，负值为上偏
                    
                    # 存储检测框左上角x坐标（用于从左到右排序）和偏差值
                    detections.append({
                        'x1': x1,
                        'x_offset': x_offset,
                        'y_offset': y_offset
                    })
            if detections:
                self.miss_count = 0
                detections.sort(key=lambda d: d['x1'])

                offsets = []
                for d in detections:
                    offsets.extend([d['x_offset'], d['y_offset']])

                # 滑动窗口只对第一个目标做平滑
                self.offset_window.append((offsets[0], offsets[1]))
                avg_x = sum(p[0] for p in self.offset_window) / len(self.offset_window)
                avg_y = sum(p[1] for p in self.offset_window) / len(self.offset_window)
                offsets[0] = avg_x
                offsets[1] = avg_y

                self.last_valid_offsets = offsets

                if abs(avg_x) < self.error and abs(avg_y) < self.error:
                    self.get_logger().info('目标已对齐，偏差在误差范围内')
                    return FetchSp.Result(success=True)

                offset_msg = Float32MultiArray()
                offset_msg.data = offsets
                self.offset_publisher.publish(offset_msg)
                self.get_logger().info(f'发布 {len(offsets)//2} 个物体的偏差: {offsets}')
            else:
                self.miss_count += 1
                if self.miss_count <= self.max_miss_frames and self.last_valid_offsets is not None:
                    offset_msg = Float32MultiArray()
                    offset_msg.data = self.last_valid_offsets
                    self.offset_publisher.publish(offset_msg)
                    self.get_logger().info('检测短时丢失，复用上一帧偏差')
                else:
                    self.offset_window.clear()
                    self.last_valid_offsets = None
                    offset_msg = Float32MultiArray()
                    offset_msg.data = []
                    self.offset_publisher.publish(offset_msg)
                    self.get_logger().warning('连续丢失超过阈值，清空偏差输出')

            if self.debug_level == 1 and results is not None:  # 调试输出
            # 发布标注后的图像
                try:
                    annotated_frame = results[0].plot()
                    # 在图像上绘制中心点
                    cv2.circle(annotated_frame, (center_x, center_y), 5, (0, 0, 255), -1)
                    cv2.line(annotated_frame, (center_x-10, center_y), (center_x+10, center_y), (0, 0, 255), 1)
                    cv2.line(annotated_frame, (center_x, center_y-10), (center_x, center_y+10), (0, 0, 255), 1)
                    
                    # 转换为ROS图像消息并发布
                    annotated_image_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding='bgr8')
                    self.annotated_image_publisher.publish(annotated_image_msg)
                except Exception as e:
                    self.get_logger().warning(f'标注图像发布失败: {e}')


        return FetchSp.Result(success=False)
        
    def goal_callback(self, goal_request):
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        return CancelResponse.ACCEPT
    
def main(args=None):
    rclpy.init(args=args)
    node = YOLOSpearheadSeeNode()
    try:
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()