#!/usr/bin/env python3
"""
简单的目标点发布脚本（独立运行，无需ROS包）
发布目标XY坐标到 /target 话题，并发布：
    - /target_heading (Int16): 0前、1左、2后、3右
    - /stair_direction (Int16): 0上台阶、1下台阶

参数通过命令行传入：
  --x <float>  目标X（单位：米，默认1.0）
  --y <float>  目标Y（单位：米，默认2.0）
    --heading <int>  目标朝向（0前/1左/2后/3右，默认0）
    --stair-direction <int>  台阶方向（0上台阶/1下台阶，默认0）
  --rate <float> 发布频率（Hz，默认10.0）
    --loop       持续循环发布（默认关闭，即只发送一次）
    --wait-subs-timeout <float> 单次模式等待订阅者超时（秒，默认1.0）
    --once-burst-duration <float> 单次模式突发发送时长（秒，默认0.3）
    --once-burst-rate <float> 单次模式突发发送频率（Hz，默认20.0）
示例：
    单次发送（默认）：
    python3 src/utils/publish_target/publish_target.py --x 1.5 --y 2.5 --heading 1 --stair-direction 0
    循环发送：
    python3 src/utils/publish_target/publish_target.py --x 1.5 --y 2.5 --heading 2 --stair-direction 1 --rate 5 --loop
"""

import argparse
import time
import rclpy
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Int16


class TargetPublisher(Node):
    def __init__(
        self,
        target_x: float,
        target_y: float,
        target_heading: int,
        stair_direction: int,
        rate_hz: float,
        loop: bool,
        wait_subs_timeout: float,
        once_burst_duration: float,
        once_burst_rate: float,
    ):
        super().__init__('target_publisher')

        # 创建发布者（与控制侧 /target 订阅 QoS 对齐）
        # RELIABLE + TRANSIENT_LOCAL + KEEP_LAST(1)
        target_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.publisher_ = self.create_publisher(Point, 'target', target_qos)
        self.heading_pub_ = self.create_publisher(Int16, 'target_heading', 10)
        self.stair_pub_ = self.create_publisher(Int16, 'stair_direction', 10)

        # 保存目标与频率（单位：米 / Hz）
        self.target_x = float(target_x)
        self.target_y = float(target_y)
        self.target_heading = max(0, min(3, int(target_heading)))
        self.stair_direction = max(0, min(1, int(stair_direction)))
        self.rate_hz = float(rate_hz)
        self.loop = bool(loop)
        self.wait_subs_timeout = max(0.0, float(wait_subs_timeout))
        self.once_burst_duration = max(0.0, float(once_burst_duration))
        self.once_burst_rate = max(1.0, float(once_burst_rate))

        # 循环模式才创建定时器；默认单次发送
        self.timer = None
        if self.loop:
            timer_period = 1.0 / self.rate_hz if self.rate_hz > 0 else 1.0
            self.timer = self.create_timer(timer_period, self.timer_callback)
            self.get_logger().info(
                f'Target Publisher initialized in LOOP mode. Publishing (x={self.target_x}, y={self.target_y}) at {self.rate_hz} Hz')
        else:
            self.get_logger().info(
                f'Target Publisher initialized in ONCE mode. '
                f'Will publish burst: (x={self.target_x}, y={self.target_y}, '
                f'heading={self.target_heading}, stair_direction={self.stair_direction}), '
                f'wait_subs_timeout={self.wait_subs_timeout}s, '
                f'burst_duration={self.once_burst_duration}s, burst_rate={self.once_burst_rate}Hz')

    def _build_msg(self) -> Point:
        msg = Point()
        msg.x = self.target_x
        msg.y = self.target_y
        msg.z = 0.0  # 未使用
        return msg

    def _build_heading_msg(self) -> Int16:
        msg = Int16()
        msg.data = self.target_heading
        return msg

    def _build_stair_msg(self) -> Int16:
        msg = Int16()
        msg.data = self.stair_direction
        return msg

    def _publish_bundle(self):
        target_msg = self._build_msg()
        heading_msg = self._build_heading_msg()
        stair_msg = self._build_stair_msg()

        self.publisher_.publish(target_msg)
        self.heading_pub_.publish(heading_msg)
        self.stair_pub_.publish(stair_msg)

        return target_msg, heading_msg, stair_msg

    def publish_once_reliable(self):
        def _subscription_count() -> int:
            # 兼容不同 ROS2 版本：部分版本没有 get_intra_process_subscription_count
            count = int(self.publisher_.get_subscription_count())
            if hasattr(self.publisher_, 'get_intra_process_subscription_count'):
                count += int(self.publisher_.get_intra_process_subscription_count())
            return count

        # 先等待订阅者发现，减少“单次发送时未匹配成功”导致的丢失
        deadline = time.monotonic() + self.wait_subs_timeout
        while time.monotonic() < deadline:
            subs = _subscription_count()
            if subs > 0:
                break
            rclpy.spin_once(self, timeout_sec=0.05)

        subs = _subscription_count()
        if subs == 0:
            self.get_logger().warning('No subscribers detected before timeout; still sending burst.')
        else:
            self.get_logger().info(f'Detected subscribers: {subs}')

        # 单次模式下做短时突发发送，提升送达概率
        count = max(1, int(round(self.once_burst_duration * self.once_burst_rate)))
        period = 1.0 / self.once_burst_rate
        msg = self._build_msg()
        for i in range(count):
            self._publish_bundle()
            rclpy.spin_once(self, timeout_sec=0.0)
            if i != count - 1:
                time.sleep(period)

        self.get_logger().info(
            f'Published target burst done: x={msg.x:.3f}, y={msg.y:.3f}, '
            f'heading={self.target_heading}, stair_direction={self.stair_direction}, count={count}')

    def timer_callback(self):
        msg, heading_msg, stair_msg = self._publish_bundle()
        self.get_logger().info(
            f'Publishing target: x={msg.x:.3f}, y={msg.y:.3f}, '
            f'heading={heading_msg.data}, stair_direction={stair_msg.data}')


def main():
    parser = argparse.ArgumentParser(description='Publish target XY to /target topic')
    parser.add_argument('--x', type=float, default=1.0, help='Target X in meters')
    parser.add_argument('--y', type=float, default=2.0, help='Target Y in meters')
    parser.add_argument('--heading', type=int, default=0,
                        help='Target heading: 0-front, 1-left, 2-back, 3-right')
    parser.add_argument('--stair-direction', type=int, default=0,
                        help='Stair direction: 0-upstairs, 1-downstairs')
    parser.add_argument('--rate', type=float, default=10.0, help='Publish rate in Hz')
    parser.add_argument('--loop', action='store_true', help='Enable continuous publishing (default: publish once)')
    parser.add_argument('--wait-subs-timeout', type=float, default=1.0,
                        help='Wait timeout for subscriber discovery in once mode (seconds)')
    parser.add_argument('--once-burst-duration', type=float, default=0.0,
                        help='Burst send duration in once mode (seconds)')
    parser.add_argument('--once-burst-rate', type=float, default=20.0,
                        help='Burst send rate in once mode (Hz)')
    cli_args = parser.parse_args()

    rclpy.init()
    node = TargetPublisher(
        cli_args.x,
        cli_args.y,
        cli_args.heading,
        cli_args.stair_direction,
        cli_args.rate,
        cli_args.loop,
        cli_args.wait_subs_timeout,
        cli_args.once_burst_duration,
        cli_args.once_burst_rate,
    )

    try:
        if cli_args.loop:
            rclpy.spin(node)
        else:
            node.publish_once_reliable()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
