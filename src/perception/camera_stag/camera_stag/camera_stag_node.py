#!/usr/bin/env python3
import cv2
import os
import re
import stag
import rclpy
import numpy as np
import threading
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Bool, Int32, String
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.action import ActionServer, CancelResponse, GoalResponse
import rclpy.action
from ali_interface.action import AliSp

class CameraStagNode(Node):
    def __init__(self):
        super().__init__('camera_stag_node')
    
        self.x_mm = 0.0
        self.y_mm = 0.0
        self.z_mm = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.tag_id = -1
        self.ACK = 0
        self.detected = False
        self.capture_event = threading.Event()
        self.data_lock = threading.Lock()
        self.last_rect  = None   # (x,y,w,h)
        self.last_cx    = -1
        self.last_cy    = -1

        # 创建回调组
        self.action_group = ReentrantCallbackGroup()
        self.sub_group = ReentrantCallbackGroup()
    
        # 发布器
        self.detect_pub = self.create_publisher(Bool, 'stag_detected', 10)
        self.id_pub     = self.create_publisher(Int32, 'stag_id', 10)
        self.cen_pub    = self.create_publisher(Point, 'stag_center', 10)
        self.image_pub  = self.create_publisher(Image, 'stag_image', 10)
        self.angle_pub  = self.create_publisher(Int32, 'stag_angle', 10)    

        self.bridge = CvBridge()
        self.libraryHD = 23

        # 订阅 terraced_camera_node 发布的图像
        self.image_sub = self.create_subscription(
            Image,
            'terraced_camera_image',
            self.image_callback,
            10,
            callback_group=self.sub_group
        )

        # 订阅ACK码
        self.ACK_sub = self.create_subscription(
            String, 
            'ACK',
            self.ACK_callback,
            10,
            callback_group=self.sub_group
        )

        # 发布动作码
        self.action_code_publisher = self.create_publisher(
            int, 
            'action_code', 
            10
        )

        # 创建动作服务器
        self.Ali_Action_Server = rclpy.action.ActionServer(
            self,
            AliSp,
            'AliSp_Action',
            execute_callback = self.execute_callback,
            goal_callback = self.goal_callback,
            cancel_callback = self.cancel_callback,
            callback_group=self.action_group
        )

        self.get_logger().info('camera_stag_node started (订阅 terraced_camera_image)')

        #STag尺寸
        self.stag_size = 0.064  # meters
        #STag相机内参(示例值)
        self.camera_matrix = np.array([[694.5642, 0.0, 364.9261],
                                       [0.0, 695.3598, 351.8180],
                                       [0.0, 0.0, 1.0]])
        self.dist_coeffs = np.array([0.122, -0.3814, 0, 0, 0], dtype=np.float32) # 畸变
        self.stag_point = np.array([[-self.stag_size/2, -self.stag_size/2, 0],
                                    [ self.stag_size/2, -self.stag_size/2, 0],
                                    [ self.stag_size/2,  self.stag_size/2, 0],
                                    [-self.stag_size/2,  self.stag_size/2, 0]], dtype=np.float32)
        # 初始化卡尔曼滤波器
        self.init_kalman()

    # ---------- 图像回调处理 ----------
    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        self.process_frame(frame)
        

    # ---------- ACK回调处理 --------
    def ACK_callback(self, msg):
        with self.data_lock:
            self.ACK = msg.data
        if self.ACK == 2:
            self.capture_event.set()  # 触发捕获事件
        else:
            self.capture_event.clear()  # 清除捕获事件

    # ---------- 主处理逻辑 ----------
    def process_frame(self, frame):
        pts, (cx, cy), detected, tag_id, color = self.detect_stag_marker(frame, self.libraryHD)
        with self.data_lock:
            self.detected = detected
            self.tag_id = tag_id
        # 画框
        if pts is not None:
            red = (0, 0, 255)
            cv2.polylines(frame, [pts], True, color, 2)
            cv2.circle(frame, (cx, cy), 5, red, -1)
            x, y, w, h = cv2.boundingRect(pts)
            self.last_rect = (x, y, w, h)
            self.last_cx, self.last_cy = cx, cy
            
            # 计算姿态
            
            retval, rvec, tvec = cv2.solvePnP(self.stag_point, pts.astype(np.float32), self.camera_matrix, self.dist_coeffs)
            if retval:
                with self.data_lock:
                    self.x_mm = tvec[0][0]*1000  # X 坐标（单位毫米）
                    self.y_mm = tvec[1][0]*1000  # Y 坐标
                    self.z_mm = tvec[2][0]*1000  # Z 坐标（深度，即距离相机光心的直线距离）
                R, _ = cv2.Rodrigues(rvec)
                sy = np.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
                singular = sy < 1e-6
                if not singular:
                    roll = np.arctan2(R[2,1] , R[2,2])
                    pitch = np.arctan2(-R[2,0], sy)
                    yaw = np.arctan2(R[1,0], R[0,0])
                else:
                    roll = np.arctan2(-R[1,2], R[1,1])
                    pitch = np.arctan2(-R[2,0], sy)
                    yaw = 0
                # 转换为度
                roll = np.degrees(roll)
                pitch = np.degrees(pitch)
                yaw = np.degrees(yaw)
                # 卡尔曼滤波
                roll = self.kalman_filter('roll', roll)
                pitch = self.kalman_filter('pitch', pitch)
                yaw = self.kalman_filter('yaw', yaw)
                with self.data_lock:
                    self.roll = roll
                    self.pitch = pitch
                    self.yaw = yaw
                
        else:
            roll = pitch = yaw = 90.1
            self.last_rect = None
            self.last_cx, self.last_cy = -1, -1
            # 未识别时不更新卡尔曼滤波器

    def detect_stag_marker(self, frame, libraryHD):
        """
        识别stag标记，返回角点、中心点、检测状态、tag_id。
        返回：pts(4x2 array or None), (cx, cy), detected(bool), tag_id(int)
        """
        blurred = cv2.GaussianBlur(frame, (3, 3), 1)
        blurred = cv2.bilateralFilter(blurred, d=9, sigmaColor=75, sigmaSpace=75)
        corners, ids, rejected = stag.detectMarkers(blurred, libraryHD)

        # 1. 正式标记
        if ids is not None and len(ids) > 0:
            pts = corners[0][0].astype(int)
            cx, cy = pts.mean(axis=0).astype(int)
            tag_id = int(ids[0, 0])
            detected = True
            color = (0, 255, 0)
            return pts, (cx, cy), detected, tag_id, color
        # 2. 用 rejected 做补救
        elif self.last_rect is not None and rejected is not None:
            lx, ly, lw, lh = self.last_rect
            last_area = lw * lh
            for r in rejected:
                pts = r[0].astype(int)
                x, y, w, h = cv2.boundingRect(pts)
                area = w * h
                if abs(area - last_area) / last_area > 0.25:
                    continue
                rx, ry = pts.mean(axis=0)
                if np.linalg.norm([rx - self.last_cx, ry - self.last_cy]) > 30:
                    continue
                cx, cy = int(rx), int(ry)
                tag_id = -2
                detected = True
                color = (255, 0, 0) 
                return pts, (cx, cy), detected, tag_id, color
        # 3. 未检测到
        return None, (-1, -1), False, -1, None

    def destroy_node(self):
        super().destroy_node()
        
    # ---------- 卡尔曼滤波器 ----------
    def init_kalman(self):
        # 一维卡尔曼滤波参数
        self.kalman = {}
        for name in ['roll', 'pitch', 'yaw']:
            self.kalman[name] = {
                'x': 0.0,      # 状态
                'P': 1.0,      # 估计协方差
                'Q': 0.1,     # 过程噪声
                'R': 2.0,      # 测量噪声
            }
    def kalman_filter(self, name, z):
        k = self.kalman[name]
        # 预测
        k['P'] += k['Q']
        # 更新
        K = k['P'] / (k['P'] + k['R'])
        k['x'] += K * (z - k['x'])
        k['P'] *= (1 - K)
        return k['x']

    def goal_callback(self, goal_request):
            self.get_logger().info('Received goal request')
            return GoalResponse.ACCEPT  

    
    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT    
    
    def execute_callback(self, goal_handle): 
        self.action_code_publisher.publish(3)    
        self.get_logger().info('Executing goal...')
        result = AliSp.Result()
        self.capture_event.clear()
        try:
            while not self.capture_event.wait(timeout=0.1):  # 等待捕获事件被触发
                if goal_handle.is_cancel_requested:  # 如果客户端请求取消，则取消目标
                    self.get_logger().info('Goal canceled by client')
                    goal_handle.canceled()
                    result.success = False
                    return result

                # 发布话题
                with self.data_lock:
                    x_mm = self.x_mm
                    y_mm = self.y_mm
                    z_mm = self.z_mm
                    detected = self.detected
                    tag_id = self.tag_id
                    pitch = self.pitch

                error = Point(x=float(x_mm), y=float(z_mm), z=float(y_mm))
                self.detect_pub.publish(Bool(data=detected))
                self.id_pub.publish(Int32(data=tag_id))
                self.cen_pub.publish(error)
                self.angle_pub.publish(Int32(data=int(pitch * 10)))
        
            self.get_logger().info('Capture event triggered, finishing goal')
            goal_handle.succeed()
            result.success = True
        except Exception as e:
            self.get_logger().error(f'Error during execute_callback: {e}')
            goal_handle.abort()
            result.success = False
        
        return result


def main(args=None):
    rclpy.init(args=args)
    node = CameraStagNode()
    try:
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
