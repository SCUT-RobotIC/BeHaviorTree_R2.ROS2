import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import trimesh
import cv2
from ultralytics import YOLO
from .estimater import *
import os
import torch
from segment_anything import sam_model_registry, SamPredictor
import numpy as np
from geometry_msgs.msg import Pose
import tf_transformations as tf
import cv_bridge
import sensor_msgs.msg
import message_filters
from .Utils import *
import imageio
import logging
from datetime import datetime
from pose_interface.srv import ArmSolve
from foundationpose_interface.srv import PoseSolve
# from trtyolo import TRTYOLO
from ament_index_python.packages import get_package_share_directory
import logging

class AutoTracker(Node):
    # realsense 相机内参
    # 640x480分辨率下的相机内参（来自/camera/aligned_depth_to_color/camera_info）
    K = [   388.20819091796875, 0.0, 321.27618408203125,
            0.0, 387.6481018066406, 247.6659393310547,
            0.0, 0.0, 1.0 ]
    K = np.array(K).reshape(3, 3)
    def __init__(self):
        super().__init__('auto_predict_node')
        self.cb_group = ReentrantCallbackGroup()
        # 初始化函数
        # 获取包的共享目录（install/pose_est/share/pose_est/）
        package_share = get_package_share_directory('pose_est')
        config_dir = os.path.join(package_share, 'config')
        
        # 统一使用标准路径
        labels_path = os.path.join(config_dir, 'labels.txt')
        self.yolo_model_path = os.path.join(config_dir, 'best.pt')
        self.mesh_path = os.path.join(config_dir, 'KFS.obj')
        self.sam_checkpoint = os.path.join(config_dir, 'sam_vit_b_01ec64.pth')
        
        # 动态读取类别文件
        try:
            with open(labels_path, 'r') as f:
                self.target_class_id = tuple(line.strip() for line in f if line.strip())
        except Exception as e:
            logging.warning(f"[AutoTracker] Failed to load labels.txt: {e}")
            self.target_class_id = ()
        
        # 声明参数（使用已解析的路径）
        self.declare_parameter('debug_level', 20)
        self.declare_parameter('yolomodel', self.yolo_model_path)
        self.declare_parameter('mesh_path', self.mesh_path)
        self.declare_parameter('sam_checkpoint', self.sam_checkpoint)
        self.declare_parameter('threshold', 0.5)
        self.declare_parameter('max_gpu_mem_gb', 6.0)
        self.declare_parameter('auto_restart', True)
        self.declare_parameter('sam_debug_dir', '~/Desktop/sec_dec/debug/pose_est')
        # 订阅 RGB 和 深度 图像话题
        self.RGB_sub = message_filters.Subscriber(
            self, 
            sensor_msgs.msg.Image, 
            '/camera/camera/color/image_raw')
        self.depth_sub = message_filters.Subscriber(
            self,
            sensor_msgs.msg.Image,
            '/camera/camera/aligned_depth_to_color/image_raw')
        # 同步器，确保RGB和depth帧同步
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.RGB_sub, self.depth_sub], queue_size=10, slop=0.1)
        self.ts.registerCallback(self.pose_track_callback)
        # 发布姿态话题
        self.foundationpose_pub = self.create_publisher(Pose, '/foundationpose/pose', 10)
        # 机械臂解算服务客户端
        self.arm_client = self.create_client(ArmSolve, 'solve_arm_ik', callback_group=self.cb_group)
        # 通讯服务
        self.comm_srv = self.create_service(PoseSolve, 'solve_pose', self.comm_trigger_callback, callback_group=self.cb_group)
        
        # CvBridge 用于 ROS 图像消息与 OpenCV 图像之间转换
        self.bridge = cv_bridge.CvBridge()
        # 读取参数值
        self.yolo_model_path = self.get_parameter('yolomodel').get_parameter_value().string_value  
        self.mesh_path = self.get_parameter('mesh_path').get_parameter_value().string_value
        self.sam_checkpoint = self.get_parameter('sam_checkpoint').get_parameter_value().string_value
        self.threshold = self.get_parameter('threshold').get_parameter_value().double_value
        # 设置日志级别，通过改变debug_level参数控制
        self.debug_level = self.get_parameter('debug_level').get_parameter_value().integer_value
        self.get_logger().set_level(self.debug_level)
        self.get_logger().info(f"日志等级设置为: {self.debug_level}")
        self.max_gpu_mem_gb = self.get_parameter('max_gpu_mem_gb').get_parameter_value().double_value
        self.auto_restart = self.get_parameter('auto_restart').get_parameter_value().bool_value
        self.sam_debug_dir = os.path.expanduser(
            self.get_parameter('sam_debug_dir').get_parameter_value().string_value
        )
        os.makedirs(self.sam_debug_dir, exist_ok=True)
        self.get_logger().info(f"SAM debug image dir: {self.sam_debug_dir}")
        # 定时器：定期监控显存
        # self.gpu_monitor_timer = self.create_timer(5.0, self.monitor_gpu_memory)
        # 初始化模型
        # if self.debug_level <= 10:
        #     self.yolomodel = TRTYOLO("yolo11n-with-plugin.engine", task="detect", profile=False, swap_rb=True) 
        # else:
        #     self.yolomodel = TRTYOLO("yolo11n-with-plugin.engine", task="detect", profile=True, swap_rb=True)
        self.yolomodel = YOLO(self.yolo_model_path)
        self.mesh = trimesh.load(self.mesh_path)
        self.scorer = ScorePredictor()
        self.refiner = PoseRefinePredictor()
        self.glctx = dr.RasterizeCudaContext()
        # SAM1 配置
        try:
            # 先检查 checkpoint 文件是否存在
            if not os.path.exists(self.sam_checkpoint):
                self.get_logger().error(f"SAM checkpoint not found: {self.sam_checkpoint}")
            # 加载 SAM 模型
            self.sam = sam_model_registry["vit_b"](checkpoint=self.sam_checkpoint)
            self.sam.to('cuda' if torch.cuda.is_available() else 'cpu')
            self.sam_predictor = SamPredictor(self.sam)
        except Exception as e:
            self.get_logger().error(f"SAM model loading failed: {e}")
            raise e
        # FoundationPose 配置
        self.est = FoundationPose(  model_pts=self.mesh.vertices, 
                                    model_normals=self.mesh.vertex_normals,
                                    mesh=self.mesh, scorer=self.scorer, 
                                    refiner=self.refiner,
                                    glctx=self.glctx)
        # 只保留检测和初始化相关变量
        self.last_pose = None
        self.to_origin, self.extents = trimesh.bounds.oriented_bounds(self.mesh)
        self.initial_pose = None # 初始化姿态
        self.latest_rgb = None
        self.latest_depth = None
        self.latest_K = self.K

    # 主回调函数，处理同步的 RGB 和 深度 图像
    def pose_track_callback(self, rgb_msg: sensor_msgs.msg.Image, depth_msg: sensor_msgs.msg.Image):
        # 只保存最新帧，等待通讯服务触发
        RGB_frame = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='rgb8')
        depth_frame = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='16UC1')
        if depth_frame.dtype == np.uint16:
            depth_frame = depth_frame.astype(np.float32) / 1000.0
        self.latest_rgb = RGB_frame
        self.latest_depth = depth_frame
        # 仅在debug模式下可视化YOLO检测结果
        if self.debug_level <= 20:  # INFO或更低
            results = self.yolomodel.predict(source=RGB_frame, verbose=False)
            merged_box, original_boxes = merge_boxes(results, self.target_class_id, self.threshold)
            debug_img = cv2.cvtColor(RGB_frame, cv2.COLOR_RGB2BGR)
            if original_boxes:
                for box in original_boxes:
                    cv2.rectangle(debug_img, (int(box[0]), int(box[1])), (int(box[2]), int(box[3])), (0, 0, 255), 2)
            if merged_box is not None:
                if isinstance(merged_box, (list, np.ndarray)) and len(merged_box) == 4:
                    cv2.rectangle(debug_img, (int(merged_box[0]), int(merged_box[1])), (int(merged_box[2]), int(merged_box[3])), (0, 255, 0), 2)
                elif isinstance(merged_box, (list, np.ndarray)) and len(merged_box) == 3:
                    for pt in merged_box:
                        cv2.circle(debug_img, (int(pt[0]), int(pt[1])), 5, (0, 255, 0), -1)
            try:
                cv2.imshow("YOLO Detection", debug_img)
                cv2.waitKey(1)
            except Exception as e:
                self.get_logger().debug(f"cv2.imshow failed: {e}")

    # 搜索预处理函数
    # 服务回调触发初始化

    def comm_trigger_callback(self, request, response):
        # 仅当request.need为True时才进行姿态结算，否则直接返回
        try:
            if hasattr(request, 'need') and request.need:
                self.get_logger().info("Received comm trigger request: need=True, running SAM+FoundationPose init...")
                if self.latest_rgb is None or self.latest_depth is None:
                    self.get_logger().warn("No image data available for initialization.")
                    response.success = False
                    return response
                # YOLO检测
                results = self.yolomodel.predict(source=self.latest_rgb, verbose=False)
                merged_box, original_boxes = merge_boxes(results, self.target_class_id, self.threshold)
                if not (merged_box and original_boxes):
                    response.success = False
                    return response
                # SAM分割
                self.sam_predictor.set_image(self.latest_rgb)
                if (len(merged_box) == 4):
                    box = np.array(merged_box)
                    masks, scores, logits = self.sam_predictor.predict(
                        box=box[None, :],
                        multimask_output=False
                    )
                elif (len(merged_box) == 3):
                    box = np.array(merged_box)
                    point_labels = np.ones(len(box), dtype=np.int32)
                    masks, scores, logits = self.sam_predictor.predict(
                        point_coords=box,
                        point_labels=point_labels,
                        multimask_output=False
                    )
                else:
                    response.success = False
                    return response
                if masks is not None and len(masks) > 0:
                    mask = masks[0]
                    mask_binary = (mask.astype(np.uint8) * 255)
                else:
                    response.success = False
                    return response
                # 可视化分割mask
                vis_img = cv2.cvtColor(self.latest_rgb, cv2.COLOR_RGB2BGR).copy()
                color_mask = np.zeros_like(vis_img)
                color_mask[mask > 0] = [0, 255, 0]
                vis_img = cv2.addWeighted(vis_img, 0.7, color_mask, 0.3, 0)
                # 保存 SAM 结果图和二值 mask 到调试目录
                try:
                    ts = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
                    vis_path = os.path.join(self.sam_debug_dir, f'sam_overlay_{ts}.png')
                    mask_path = os.path.join(self.sam_debug_dir, f'sam_mask_{ts}.png')
                    cv2.imwrite(vis_path, vis_img)
                    cv2.imwrite(mask_path, mask_binary)
                    self.get_logger().info(f"Saved SAM debug images: {vis_path}, {mask_path}")
                except Exception as e:
                    self.get_logger().warning(f"Failed to save SAM debug images: {e}")
                # FoundationPose初始化
                pose = self.est.register(K=self.latest_K, rgb=self.latest_rgb, depth=self.latest_depth, ob_mask=mask_binary, iteration=5)
                if pose is not None:
                    self.last_pose = pose
                    pos_msg = matrix_to_pose_msg(pose)
                    self.foundationpose_pub.publish(pos_msg)
                    self.get_logger().info(
                        f"Published pose: p=({pos_msg.position.x:.4f}, {pos_msg.position.y:.4f}, {pos_msg.position.z:.4f})"
                    )
                    # 保存带位姿框的可视化图，便于离线回放与标注核对
                    try:
                        pose_vis_img = cv2.cvtColor(self.latest_rgb, cv2.COLOR_RGB2BGR).copy()
                        bbox = np.stack([-self.extents / 2, self.extents / 2], axis=0).reshape(2, 3)
                        pose_vis_img = draw_posed_3d_box(self.latest_K, pose_vis_img, pose, bbox)
                        pose_ts = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
                        pose_path = os.path.join(self.sam_debug_dir, f'pose_box_{pose_ts}.png')
                        cv2.imwrite(pose_path, pose_vis_img)
                        self.get_logger().info(f"Saved pose box image: {pose_path}")
                    except Exception as e:
                        self.get_logger().warning(f"Failed to save pose box image: {e}")
                    # 请求机械臂逆解服务，参数用姿态解算结果
                    arm_request = ArmSolve.Request()
                    arm_request.x = pos_msg.position.x
                    arm_request.y = pos_msg.position.y
                    arm_request.z = pos_msg.position.z
                    if not self.arm_client.wait_for_service(timeout_sec=0.5):
                        self.get_logger().warning('solve_arm_ik service is not available, skip IK call this cycle.')
                        response.success = False
                        return response
                    arm_response = self.arm_client.call(arm_request)
                    response.success = arm_response.success
                    response.theta_sum1 = arm_response.theta_sum1
                    response.theta_sum2 = arm_response.theta_sum2
                    response.theta_sum3 = arm_response.theta_sum3
                    response.theta_sum_yaw = arm_response.theta_sum_yaw
                else:
                    response.success = False
            else:
                self.get_logger().info("Received comm trigger request: need=False, skip pose estimation.")
                response.success = True
            return response
        finally:
            torch.cuda.empty_cache()    
    
    # 释放资源，避免内存泄漏，显存碎片化
    def destroy_node(self):
        # 显式释放大模型和 GPU 资源
        try:
            if hasattr(self, 'sam_predictor'):
                del self.sam_predictor
            torch.cuda.empty_cache()
            if rclpy.ok():
                self.get_logger().info("Released GPU and model resources.")
            del self.est
            del self.yolomodel
        except Exception as e:
            if rclpy.ok():
                self.get_logger().warning(f"Resource release failed: {e}")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = None
    executor = None
    try:
        node = AutoTracker()
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if executor is not None:
            try:
                executor.shutdown()
            except Exception:
                pass
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                # 退出阶段尽量不抛异常，避免覆盖主异常
                pass
        # Ctrl-C 场景下 context 可能已被外部流程关闭，这里仅在有效时 shutdown
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()