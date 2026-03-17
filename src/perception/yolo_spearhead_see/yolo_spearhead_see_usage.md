# yolo_spearhead_see 包说明

## 简介
`yolo_spearhead_see` 是一个基于 YOLOv8 的 ROS2 感知节点，用于识别摄像头画面中的矛头目标，并输出目标相对于图像中心的像素偏差，同时发布带有检测框和中心点标注的图像。

## 节点功能
- 订阅 `/terraced_camera_image` 话题（`sensor_msgs/msg/Image`），获取摄像头图像。
- 使用 YOLOv8 模型进行目标检测。
- 计算每个检测目标中心点相对于图像中心的像素偏差（x、y），并按从左到右顺序输出。
- 发布所有检测目标的偏差数组到 `/yolo_detection_offsets`（`std_msgs/Float32MultiArray`）。
- 发布带有检测框和中心点标注的图像到 `/yolo_annotated_image`（`sensor_msgs/msg/Image`）。

## 主要话题
| 话题名称                | 消息类型                    | 说明                         |
|------------------------|----------------------------|------------------------------|
| /terraced_camera_image | sensor_msgs/msg/Image      | 输入，摄像头原始图像         |
| /yolo_detection_offsets| std_msgs/Float32MultiArray | 输出，所有目标的像素偏差数组 |
| /yolo_annotated_image  | sensor_msgs/msg/Image      | 输出，带检测框的标注图像     |

## 偏差数组说明
- 每个目标输出 `[x_offset, y_offset]`，单位为像素，顺序为从左到右。
- `x_offset > 0` 表示目标在图像中心右侧，`< 0` 表示左侧。
- `y_offset > 0` 表示目标在图像中心下方，`< 0` 表示上方。

## 启动方法
1. 确保 `/terraced_camera_image` 话题已由摄像头节点发布。
2. 启动本节点：
   ```bash
   ros2 run yolo_spearhead_see yolo_spearhead_see
   ```
3. 检查 `/yolo_detection_offsets` 和 `/yolo_annotated_image` 话题是否有数据。

## 依赖
- ROS2 Humble
- ultralytics (YOLOv8)
- OpenCV
- numpy
- cv_bridge

## 典型应用场景
- 机器人矛头对接、目标跟踪、视觉伺服等。

---
如需自定义模型，请修改节点内模型路径并确保模型文件存在。
