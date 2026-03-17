# terraced_camera 包说明

## 启动的串口
本包通过 `HBSCameraNode` 节点自动查找并打开 `/dev/v4l/by-id` 目录下名称包含 `device_name_keyword`（默认 `HBS_Camera`）且以 `index0` 结尾的摄像头设备。实际打开的设备路径如 `/dev/videoX`，通过 OpenCV 的 `cv2.VideoCapture` 进行访问。

## 发布的话题
- **话题名称**：`/terraced_camera_image`
- **消息类型**：`sensor_msgs/msg/Image`
- **内容**：发布摄像头采集到的图像帧，支持图像翻转。

## 图像翻转的启动方法
1. **参数控制**：
   - `flip_mode` 参数决定图像是否翻转。
     - `0`：完全不翻转（默认值）
     - `1`：水平垂直翻转
2. **启动并翻转图像**：
   - 启动节点时通过参数设置：
     ```bash
     ros2 run terraced_camera terraced_camera_node --ros-args -p flip_mode:=1
     ```
   - 或者在运行时通过动态参数修改：
     ```bash
     ros2 param set /hbs_camera_node flip_mode 1
     ```

## 其他参数
- `publish_fps`：发布帧率，范围 1-30，默认 15。
- `device_name_keyword`：设备名匹配关键字，默认 `HBS_Camera`。
  - 如果你的设备名是 `usb-LRCP_imx415_...-video-index0`，启动时可设置：
    ```bash
    ros2 run terraced_camera terraced_camera_node --ros-args -p device_name_keyword:=LRCP_imx415
    ```

## 资源释放
节点关闭时会自动释放摄像头资源。

## Debug流程
- 检查该摄像头是否有插到USB总线上
```
lsusb | grep "HBS Camera"
```

如果无输出，请检查摄像头的连接

- 如果确认连接无误后还是找不到，插拔摄像头查看系统日志

在摄像头未出现时，打开终端运行以下命令，然后重新插拔一次 HBS 摄像头，观察实时输出的日志：

```
sudo dmesg -w
```