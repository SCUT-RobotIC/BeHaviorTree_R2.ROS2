#!/usr/bin/env python3
"""
基于 Point-LIO TF 的当前位置 + 用户输入增量，计算并发布目标点到 /target。

流程：
1) 读取 TF（默认 lidar_odom -> lidar_link）得到当前 x,y
2) 从终端读取用户输入 dx,dy（单位：米）
3) 计算 target = current + delta，并发布 geometry_msgs/Point
"""

import argparse
import time

import rclpy
from geometry_msgs.msg import Point
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import Buffer, TransformException, TransformListener


class DeltaTargetPublisherFromTF(Node):
    def __init__(
        self,
        target_topic: str,
        tf_parent_frame: str,
        tf_child_frame: str,
        tf_lookup_timeout: float,
    ) -> None:
        super().__init__('delta_target_publisher_from_pointlio_tf')

        self.tf_parent_frame = tf_parent_frame
        self.tf_child_frame = tf_child_frame
        self.tf_lookup_timeout = max(0.001, float(tf_lookup_timeout))

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        target_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.target_pub = self.create_publisher(Point, target_topic, target_qos)

        self.get_logger().info(
            f'started: target_topic={target_topic}, tf={self.tf_parent_frame}->{self.tf_child_frame}'
        )

    def get_current_xy_from_tf(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.tf_parent_frame,
                self.tf_child_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=self.tf_lookup_timeout),
            )
            return tf.transform.translation.x, tf.transform.translation.y
        except TransformException as e:
            self.get_logger().warning(f'TF lookup failed: {e}')
            return None

    def wait_for_tf(self, timeout_sec: float) -> bool:
        deadline = time.monotonic() + max(0.0, timeout_sec)
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.get_current_xy_from_tf() is not None:
                return True
        return self.get_current_xy_from_tf() is not None

    def publish_target(self, x: float, y: float, count: int = 1, rate_hz: float = 10.0) -> None:
        msg = Point()
        msg.x = float(x)
        msg.y = float(y)
        msg.z = 0.0

        count = max(1, int(count))
        period = 1.0 / max(rate_hz, 1e-6)

        for i in range(count):
            self.target_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.0)
            if i != count - 1:
                time.sleep(period)


def _read_delta_from_user():
    while True:
        raw = input('请输入 dx dy（单位米，例如: 0.2 -0.1）: ').strip()
        parts = raw.replace(',', ' ').split()
        if len(parts) != 2:
            print('输入格式错误，请输入两个数字，例如: 0.2 -0.1')
            continue
        try:
            dx = float(parts[0])
            dy = float(parts[1])
            return dx, dy
        except ValueError:
            print('输入必须是数字，请重试。')


def main() -> None:
    parser = argparse.ArgumentParser(description='Publish target as Point-LIO TF pose + user delta.')
    parser.add_argument('--target-topic', default='/target', help='Target topic (Point)')
    parser.add_argument('--tf-parent-frame', default='lidar_odom', help='TF parent frame')
    parser.add_argument('--tf-child-frame', default='lidar_link', help='TF child frame')
    parser.add_argument('--tf-lookup-timeout', type=float, default=0.05, help='Single TF lookup timeout in seconds')
    parser.add_argument('--wait-tf-timeout', type=float, default=3.0, help='Seconds to wait for initial TF')
    parser.add_argument('--publish-count', type=int, default=1, help='How many times to publish target')
    parser.add_argument('--publish-rate', type=float, default=10.0, help='Publish rate when count > 1')
    args = parser.parse_args()

    rclpy.init()
    node = DeltaTargetPublisherFromTF(
        target_topic=args.target_topic,
        tf_parent_frame=args.tf_parent_frame,
        tf_child_frame=args.tf_child_frame,
        tf_lookup_timeout=args.tf_lookup_timeout,
    )

    try:
        if not node.wait_for_tf(args.wait_tf_timeout):
            node.get_logger().error('未在超时时间内获取到 Point-LIO TF，退出。')
            return

        current = node.get_current_xy_from_tf()
        if current is None:
            node.get_logger().error('读取当前 TF 位置失败，退出。')
            return

        cx, cy = current
        node.get_logger().info(f'当前坐标(TF): x={cx:.3f}, y={cy:.3f}')

        dx, dy = _read_delta_from_user()
        tx = cx + dx
        ty = cy + dy

        node.get_logger().info(
            f'计算目标: current=({cx:.3f},{cy:.3f}) + delta=({dx:.3f},{dy:.3f}) => target=({tx:.3f},{ty:.3f})'
        )

        node.publish_target(tx, ty, count=args.publish_count, rate_hz=args.publish_rate)
        node.get_logger().info('目标发布完成。')

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
