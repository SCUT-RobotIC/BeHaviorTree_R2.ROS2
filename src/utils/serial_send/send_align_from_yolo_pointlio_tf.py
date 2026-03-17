#!/usr/bin/env python3
"""按 stm32_control 协议，从话题采集数据并持续发送 STM32 数据包。

说明：
- 除位置外，所有字段都来自 ROS 话题。
- `x_real/y_real` 改为来自 Point-LIO 的 TF（默认 `lidar_odom -> lidar_link`）。
- `action_code` 由用户通过参数自定义。
- 封包格式对齐 stm32_control 的 serialize_packet（64 字节发送包）。
"""

import struct
import threading

import rclpy
from geometry_msgs.msg import Point
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import Bool, Float32MultiArray, Float64MultiArray, UInt8MultiArray
from tf2_ros import Buffer, TransformException, TransformListener

HEADER = (0xAA, 0xAA)
FOOTER = (0xBB, 0xBB)
SEND_PACKET_SIZE = 64


class TopicPacketSender(Node):
    def __init__(self) -> None:
        super().__init__('topic_packet_sender_pointlio_tf')

        # ===== 可配置参数 =====
        self.write_topic = self.declare_parameter('write_topic', 'stm32/write').value
        self.target_topic = self.declare_parameter('target_topic', '/target').value
        self.stag_center_topic = self.declare_parameter('stag_center_topic', '/stag_center').value
        self.stag_detected_topic = self.declare_parameter('stag_detected_topic', '/stag_detected').value
        self.arm_angles_topic = self.declare_parameter('arm_angles_topic', '/arm_angles').value
        self.yolo_offsets_topic = self.declare_parameter('yolo_offsets_topic', '/yolo_detection_offsets').value

        # Point-LIO TF: parent -> child，取 child 在 parent 下的位置作为 x_real/y_real
        self.tf_parent_frame = self.declare_parameter('tf_parent_frame', 'lidar_odom').value
        self.tf_child_frame = self.declare_parameter('tf_child_frame', 'lidar_link').value
        self.tf_lookup_timeout_sec = float(self.declare_parameter('tf_lookup_timeout_sec', 0.05).value)
        self.tf_timeout_sec = float(self.declare_parameter('tf_timeout_sec', 0.5).value)

        self.action_code = int(self.declare_parameter('action_code', 3).value)
        self.rate_hz = float(self.declare_parameter('rate_hz', 50.0).value)
        self.stag_timeout_sec = float(self.declare_parameter('stag_timeout_sec', 0.5).value)
        self.arm_timeout_sec = float(self.declare_parameter('arm_timeout_sec', 1.0).value)
        self.yolo_timeout_sec = float(self.declare_parameter('yolo_timeout_sec', 0.5).value)

        # ===== 缓存数据（位置由 TF 更新，其余由话题更新） =====
        self.x_real = 0
        self.y_real = 0
        self.pose_last_update = self.get_clock().now()

        self.x_target = 0
        self.y_target = 0

        self.stag_x = 0
        self.stag_y = 0
        self.stag_detected = 0
        self.stag_last_update = self.get_clock().now()

        self.arm_joint1 = 0
        self.arm_joint2 = 0
        self.arm_joint3 = 0
        self.arm_yaw = 0
        self.arm_last_update = self.get_clock().now()

        self.x_offset = 0
        self.y_offset = 0
        self.yolo_last_update = self.get_clock().now()

        self.lock = threading.Lock()

        self.pub = self.create_publisher(UInt8MultiArray, self.write_topic, 10)

        # TF listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # target: Point，映射为 x_target/y_target（米转毫米）
        self.target_sub = self.create_subscription(
            Point,
            self.target_topic,
            self._on_target,
            10,
        )

        # stag center: Point，映射为 stag_x/stag_y（按 stm32_control 直接取整）
        self.stag_center_sub = self.create_subscription(
            Point,
            self.stag_center_topic,
            self._on_stag_center,
            10,
        )

        # stag detected: Bool -> 0/1
        self.stag_detected_sub = self.create_subscription(
            Bool,
            self.stag_detected_topic,
            self._on_stag_detected,
            10,
        )

        # arm angles: Float64MultiArray[4]，按 stm32_control 乘 10
        self.arm_angles_sub = self.create_subscription(
            Float64MultiArray,
            self.arm_angles_topic,
            self._on_arm_angles,
            10,
        )

        # yolo offsets: Float32MultiArray[>=2]
        self.yolo_offsets_sub = self.create_subscription(
            Float32MultiArray,
            self.yolo_offsets_topic,
            self._on_yolo_offsets,
            10,
        )

        self.timer = self.create_timer(1.0 / max(self.rate_hz, 1e-6), self._send_packet)

        self.get_logger().info(
            'started: '
            f'write={self.write_topic}, action_code={self.action_code}, rate={self.rate_hz:.1f}Hz, '
            f'tf={self.tf_parent_frame}->{self.tf_child_frame}'
        )

    def _on_target(self, msg: Point) -> None:
        with self.lock:
            self.x_target = self._clamp_i16(int(round(msg.x * 1000.0)))
            self.y_target = self._clamp_i16(int(round(msg.y * 1000.0)))

    def _on_stag_center(self, msg: Point) -> None:
        with self.lock:
            self.stag_x = self._clamp_i16(int(round(msg.x)))
            self.stag_y = self._clamp_i16(int(round(msg.y)))
            self.stag_last_update = self.get_clock().now()

    def _on_stag_detected(self, msg: Bool) -> None:
        with self.lock:
            self.stag_detected = 1 if msg.data else 0

    def _on_arm_angles(self, msg: Float64MultiArray) -> None:
        if len(msg.data) < 4:
            return
        with self.lock:
            self.arm_joint1 = self._clamp_i16(int(round(msg.data[0] * 10.0)))
            self.arm_joint2 = self._clamp_i16(int(round(msg.data[1] * 10.0)))
            self.arm_joint3 = self._clamp_i16(int(round(msg.data[2] * 10.0)))
            self.arm_yaw = self._clamp_i16(int(round(msg.data[3] * 10.0)))
            self.arm_last_update = self.get_clock().now()

    def _on_yolo_offsets(self, msg: Float32MultiArray) -> None:
        if len(msg.data) < 2:
            return
        with self.lock:
            self.x_offset = self._clamp_i16(int(round(msg.data[0])))
            self.y_offset = self._clamp_i16(int(round(msg.data[1])))
            self.yolo_last_update = self.get_clock().now()

    def _update_pose_from_tf(self) -> None:
        try:
            tf = self.tf_buffer.lookup_transform(
                self.tf_parent_frame,
                self.tf_child_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=self.tf_lookup_timeout_sec),
            )
            with self.lock:
                self.x_real = self._clamp_i16(int(round(tf.transform.translation.x * 1000.0)))
                self.y_real = self._clamp_i16(int(round(tf.transform.translation.y * 1000.0)))
                self.pose_last_update = self.get_clock().now()
        except TransformException:
            # TF 短时不可用时保留上次值，超时后在打包阶段归零
            pass

    def _send_packet(self) -> None:
        self._update_pose_from_tf()
        now = self.get_clock().now()
        with self.lock:
            pose_age = (now - self.pose_last_update).nanoseconds / 1e9
            stag_age = (now - self.stag_last_update).nanoseconds / 1e9
            arm_age = (now - self.arm_last_update).nanoseconds / 1e9
            yolo_age = (now - self.yolo_last_update).nanoseconds / 1e9

            x_real = self.x_real if pose_age <= self.tf_timeout_sec else 0
            y_real = self.y_real if pose_age <= self.tf_timeout_sec else 0

            stag_x = self.stag_x if stag_age <= self.stag_timeout_sec and self.stag_detected == 1 else 0
            stag_y = self.stag_y if stag_age <= self.stag_timeout_sec and self.stag_detected == 1 else 0
            stag_detected = self.stag_detected if stag_age <= self.stag_timeout_sec else 0

            arm_joint1 = self.arm_joint1 if arm_age <= self.arm_timeout_sec else 0
            arm_joint2 = self.arm_joint2 if arm_age <= self.arm_timeout_sec else 0
            arm_joint3 = self.arm_joint3 if arm_age <= self.arm_timeout_sec else 0
            arm_yaw = self.arm_yaw if arm_age <= self.arm_timeout_sec else 0

            x_offset = self.x_offset if yolo_age <= self.yolo_timeout_sec else 0
            y_offset = self.y_offset if yolo_age <= self.yolo_timeout_sec else 0

            fields = [
                self._clamp_i16(self.action_code),
                x_real,
                y_real,
                self.x_target,
                self.y_target,
                stag_x,
                stag_y,
                stag_detected,
                arm_joint1,
                arm_joint2,
                arm_joint3,
                arm_yaw,
                x_offset,
                y_offset,
            ]

        packet = self._build_packet(fields)
        out = UInt8MultiArray()
        out.data = list(packet)
        self.pub.publish(out)

    def _build_packet(self, fields: list[int]) -> bytearray:
        if len(fields) != 14:
            raise ValueError('fields length must be 14')

        buf = bytearray(SEND_PACKET_SIZE)
        buf[0], buf[1] = HEADER

        idx = 2
        for value in fields:
            b = struct.pack('<h', int(value))
            buf[idx] = b[0]
            buf[idx + 1] = b[1]
            idx += 2

        # reserved[30:62] 默认为 0
        buf[62], buf[63] = FOOTER
        return buf

    @staticmethod
    def _clamp_i16(v: int) -> int:
        return max(-32768, min(32767, v))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TopicPacketSender()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
