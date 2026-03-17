import rclpy
from rclpy.node import Node
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
from pose_interface.srv import ArmSolve
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
        self.arm_client = self.create_client(ArmSolve, 'solve_arm_ik')
        
        # CvBridge 用于 ROS 图像消息与 OpenCV 图像之间转换
        self.bridge = cv_bridge.CvBridge()
        # 读取参数值
        self.yolo_model_path = self.get_parameter('yolomodel').get_parameter_value().string_value  
        self.mesh_path = self.get_parameter('mesh_path').get_parameter_value().string_value
        self.sam_checkpoint = self.get_parameter('sam_checkpoint').get_parameter_value().string_value
        self.threshold = self.get_parameter('threshold').get_parameter_value().double_value
        # 设置日志级别，通过改变debug_level参数控制
        self.get_logger().set_level(debug_level := self.get_parameter('debug_level').get_parameter_value().integer_value)
        # 初始化模型
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
        # 状态机: SEARCHING -> TRACKING，初始为 SEARCHING
        self.state = "SEARCHING"  
        self.last_pose = None
        self.track_loss_count = 0 # 连续跟踪失败计数，超过阈值切换回 SEARCHING
        self.to_origin, self.extents = trimesh.bounds.oriented_bounds(self.mesh)
        self.track = False 
        self.initial_pose = None # 初始化姿态

    # 主回调函数，处理同步的 RGB 和 深度 图像
    def pose_track_callback(self, rgb_msg: sensor_msgs.msg.Image, depth_msg: sensor_msgs.msg.Image):
        # 1. 获取 RGB 图像
        self.get_logger().info("Received synchronized RGB and Depth frames.")
        # 从ROS消息获取RGB图像 (指定为rgb8格式)
        RGB_frame = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='rgb8')
        print("RGB_frame shape:", RGB_frame.shape)
        # 2. 获取深度图像
        depth_frame = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='16UC1')
        print("depth_frame shape:", depth_frame.shape)
        # 转换为float32并从毫米转为米
        if depth_frame.dtype == np.uint16:
            depth_frame = depth_frame.astype(np.float32) / 1000.0
            print("depth_frame converted to meters, dtype:", depth_frame.dtype)

        if self.state == "TRACKING" and self.last_pose is not None:
            self.track_frame(RGB_frame, depth_frame, self.K, self.last_pose)

        if self.state == "SEARCHING":
            self.process_frame(RGB_frame, depth_frame, self.K)

        # 发布姿态信息前判断 self.last_pose 是否为 None
        if self.last_pose is not None:
            pos = matrix_to_pose_msg(self.last_pose)
            self.foundationpose_pub.publish(pos)
            # 请求机械臂解算
            self.call_arm_solve_service(pos)
        else:
             self.get_logger().warn("No valid pose to publish.")

        # 可视化当前帧姿态
        if self.last_pose is not None:
            bbox = np.stack([-self.extents/2, self.extents/2], axis=0).reshape(2,3)
            # 可视化需要BGR格式，将RGB转回BGR
            vis_img_bgr = cv2.cvtColor(RGB_frame, cv2.COLOR_RGB2BGR)
            vis_img = draw_posed_3d_box(np.array(self.K).reshape(3,3), vis_img_bgr, self.last_pose, bbox)
            vis_img = draw_xyz_axis(vis_img, self.last_pose, scale=0.1, K=np.array(self.K).reshape(3,3), thickness=3, transparency=0, is_input_rgb=False)
            cv2.imshow('FoundationPose Tracking', vis_img)
            cv2.waitKey(1)
            # 如需保存图片，可取消下一行注释
            # imageio.imwrite('/tmp/foundationpose_vis.png', vis_img[...,::-1])

    # 搜索预处理函数
    def process_frame(self, RGB_frame, depth_frame, K):
        self.get_logger().info("Processing frame in SEARCHING mode.")
        pose = None
        mask_binary = None
        self.found = False
        import time
        
        # YOLO 推理，获得的results可能是单个结果也可能是列表
        results = self.yolomodel(RGB_frame, verbose=False)
        # print(results)

        # 如果不是list，则转换为list以统一处理
        if not isinstance(results, list):
            results = [results]
        
        # 优化：标记SAM是否已提取特征，避免对同一帧多次提取
        sam_features_extracted = False

        # 第一个变量接收融合框或者点集合，第二个变量接收所有原始框
        merged_box, original_boxes = merge_boxes(results, self.target_class_id, self.threshold)
        if merged_box is not None:
            print("Merged box or points:", len(merged_box))
        # 只用融合框做一次SAM分割
        if merged_box and original_boxes is not None:
            ts = time.time()
            # 调试模式下显示所有原始框和融合框
            if self.get_logger().get_effective_level() == rclpy.logging.LoggingSeverity.DEBUG:
                debug_img = cv2.cvtColor(RGB_frame, cv2.COLOR_RGB2BGR)
                # 画原始框（红色）
                for box in original_boxes:
                    cv2.rectangle(debug_img, (int(box[0]), int(box[1])), (int(box[2]), int(box[3])), (0, 0, 255), 2)
                # 画融合框（绿色）
                # 如果 merged_box 是4个数的框
                if isinstance(merged_box, (list, np.ndarray)) and len(merged_box) == 4 and all(isinstance(x, (int, float, np.integer, np.floating)) for x in merged_box):
                    cv2.rectangle(debug_img, (int(merged_box[0]), int(merged_box[1])), (int(merged_box[2]), int(merged_box[3])), (0, 255, 0), 2)
                # 如果 merged_box 是3个点，画点
                elif isinstance(merged_box, (list, np.ndarray)) and len(merged_box) == 3 and all(isinstance(x, (list, np.ndarray)) and len(x) == 2 for x in merged_box):
                    for pt in merged_box:
                        cv2.circle(debug_img, (int(pt[0]), int(pt[1])), 5, (0, 255, 0), -1)
                cv2.imshow("YOLO Search", debug_img)
                cv2.waitKey(1)
                cv2.imwrite(f'/tmp/fp_debug2/yolo_{ts}.png', debug_img)
            # 仅在第一次需要时提取图像特征
            if not sam_features_extracted:
                self.sam_predictor.set_image(RGB_frame)
                sam_features_extracted = True
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
                    point_coords = box,
                    point_labels=point_labels,
                    multimask_output=False
                )
                if masks is not None and len(masks) > 0:
                    print("SAM masks shape:", masks.shape)
            else:
                self.get_logger().info("Merged box has invalid format for SAM.")
                masks = None
            if masks is not None and len(masks) > 0:
                mask = masks[0]  # (H, W) bool
                print("SAM mask shape:", mask.shape)
                mask_binary = (mask.astype(np.uint8) * 255)
                # for box in original_boxes:
                #     cv2.rectangle(mask_binary, (int(box[0]), int(box[1])), (int(box[2]), int(box[3])), 255, -1)
                valid_depth = (depth_frame > 0)
                valid_mask_and_depth = np.count_nonzero((mask_binary > 0) & valid_depth)
                print("mask & valid depth overlap:", valid_mask_and_depth)
                if self.get_logger().get_effective_level() == rclpy.logging.LoggingSeverity.DEBUG:
                    cv2.imwrite(f'/tmp/fp_debug2/mask_{ts}.png', mask_binary)
                self.found = True
            else:
                self.get_logger().info("SAM did not return any masks.")
        else:
            self.get_logger().info("No merged box available for SAM.")

        if self.found and mask_binary is not None:
            self.get_logger().info("Attempting FoundationPose registration.")
            pose = self.est.register(K=K, rgb=RGB_frame, depth=depth_frame, ob_mask=mask_binary, iteration=5)
            # register成功
            if pose is not None:
                self.last_pose = pose
                self.state = "TRACKING"
                self.track_loss_count = 0
            # register失败
            else:
                self.get_logger().info("FoundationPose initialization failed. Remain in SEARCHING mode.")
                self.state = "SEARCHING"
                self.last_pose = None
            # else:
            #     self.get_logger().warn("No valid mask or object found in process_frame.")
        else:
            self.get_logger().warn("process_frame called in non-SEARCHING state.")
    
    # 姿态跟踪函数
    def track_frame(self, RGB_frame, depth_frame, K, initial_pose):
        self.get_logger().info("Tracking frame in TRACKING mode.")
        if initial_pose is None:
            self.get_logger().warn("track_frame called with None initial_pose. Skipping tracking.")
            return None
        
        pose = self.est.track_one(rgb=RGB_frame, depth=depth_frame, K=K, iteration=2)
        
        if pose is None:
            self.track_loss_count += 1
            if self.track_loss_count > 5:
                self.get_logger().info("Lost track of the object. Switching to SEARCHING mode.")
                self.state = "SEARCHING"
                self.last_pose = None
                torch.cuda.empty_cache()
            else:
                self.track_loss_count = 0
                self.last_pose = pose
        else:
            self.last_pose = pose
            self.track_loss_count = 0
    
    # 调用机械臂解算服务
    def call_arm_solve_service(self, pose_msg):
        if not self.arm_client.service_is_ready():
            # 避免刷屏，可以用 debug
            self.get_logger().debug('Arm service not ready, skipping...')
            return
            
        request = ArmSolve.Request()
        request.x = pose_msg.position.x
        request.y = pose_msg.position.y
        request.z = pose_msg.position.z
        
        future = self.arm_client.call_async(request)
        future.add_done_callback(self.arm_solve_done_callback)
        
    def arm_solve_done_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'Arm IK Solved: {response.message}')
            else:
                self.get_logger().warn(f'Arm IK Failed: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
    
    # 释放资源，避免内存泄漏，显存碎片化
    def destroy_node(self):
        # 显式释放大模型和 GPU 资源
        try:
            if hasattr(self, 'sam_predictor'):
                del self.sam_predictor
            torch.cuda.empty_cache()
            self.get_logger().info("Released GPU and model resources.")
            del self.est
            del self.yolomodel
        except Exception as e:
            self.get_logger().warn(f"Resource release failed: {e}")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = AutoTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()