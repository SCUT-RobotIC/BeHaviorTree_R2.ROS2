#!/usr/bin/env python3
"""
基于当前位置 + 用户输入增量，计算并发布目标点到 /target。

流程：
1) 订阅机器人当前位置（默认 /glim_ros/pose_corrected, PoseStamped）
2) 从终端读取用户输入的 dx, dy（单位：米）
3) 计算 target = current + delta，并发布 geometry_msgs/Point
"""

import argparse
import threading
import time

import rclpy
from geometry_msgs.msg import Point, PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


class DeltaTargetPublisher(Node):
    def __init__(self, pose_topic: str, target_topic: str) -> None:
        super().__init__('delta_target_publisher')

        self._lock = threading.Lock()
        self._latest_pose = None

        self._pose_sub = self.create_subscription(
            PoseStamped,
            pose_topic,
            self._on_pose,
            10,
        )

        # 与控制侧建议一致：可靠 + 瞬态本地（latched语义）
        target_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._target_pub = self.create_publisher(Point, target_topic, target_qos)

        self.get_logger().info(f'pose_topic={pose_topic}, target_topic={target_topic}')

    def _on_pose(self, msg: PoseStamped) -> None:
        with self._lock:
            self._latest_pose = msg

    def get_latest_xy(self):
        with self._lock:
            if self._latest_pose is None:
                return None
            return self._latest_pose.pose.position.x, self._latest_pose.pose.position.y

    def wait_for_pose(self, timeout_sec: float) -> bool:
        deadline = time.monotonic() + max(0.0, timeout_sec)
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.get_latest_xy() is not None:
                return True
        return self.get_latest_xy() is not None

    def publish_target(self, x: float, y: float, count: int = 1, rate_hz: float = 10.0) -> None:
        msg = Point()
        msg.x = float(x)
        msg.y = float(y)
        msg.z = 0.0

        count = max(1, int(count))
        period = 1.0 / max(rate_hz, 1e-6)

        for i in range(count):
            self._target_pub.publish(msg)
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
    parser = argparse.ArgumentParser(description='Publish target as current pose + user delta.')
    parser.add_argument('--pose-topic', default='/glim_ros/pose_corrected', help='Current pose topic (PoseStamped)')
    parser.add_argument('--target-topic', default='/target', help='Target topic (Point)')
    parser.add_argument('--wait-pose-timeout', type=float, default=3.0, help='Seconds to wait for initial pose')
    parser.add_argument('--publish-count', type=int, default=1, help='How many times to publish target')
    parser.add_argument('--publish-rate', type=float, default=10.0, help='Publish rate when count > 1')
    args = parser.parse_args()

    rclpy.init()
    node = DeltaTargetPublisher(args.pose_topic, args.target_topic)

    try:
        if not node.wait_for_pose(args.wait_pose_timeout):
            node.get_logger().error('未在超时时间内收到当前位置，退出。')
            return

        current = node.get_latest_xy()
        assert current is not None
        cx, cy = current
        node.get_logger().info(f'当前坐标: x={cx:.3f}, y={cy:.3f}')

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
