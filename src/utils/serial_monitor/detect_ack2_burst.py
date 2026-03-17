#!/usr/bin/env python3
"""检测接收包中 ack=2 的短时突发事件。

功能：
- 订阅 RX 话题（默认 stm32/read）。
- 当 ack=2 在窗口时间内出现次数 >= 阈值（默认 0.2s 内 >=2 次）时报警。
- 输出检测时刻附近的格式化包数据（前 N 条 + 触发后 N 条）。
"""

import argparse
import collections
import datetime as _dt
import struct
from dataclasses import dataclass
from typing import Deque, Dict, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray

HEADER = (0xAA, 0xAA)
FOOTER = (0xBB, 0xBB)


@dataclass
class PacketView:
    ts_monotonic: float
    ts_text: str
    raw_len: int
    line: str
    ack: Optional[int]


def _unpack_i16(data: bytes, offset: int) -> int:
    return struct.unpack_from('<h', data, offset)[0]


def _decode_rx_packet(data: bytes) -> Optional[Dict[str, int]]:
    # 对齐当前项目 RX 布局（至少需要读到 y_offset 的偏移 34）
    if len(data) < 36:
        return None
    try:
        return {
            'action': _unpack_i16(data, 2),
            'ack_flag': _unpack_i16(data, 4),
            'x_real': _unpack_i16(data, 6),
            'y_real': _unpack_i16(data, 8),
            'x_target': _unpack_i16(data, 10),
            'y_target': _unpack_i16(data, 12),
            'target_heading': _unpack_i16(data, 14),
            'stair_direction': _unpack_i16(data, 16),
            'stag_x': _unpack_i16(data, 18),
            'stag_y': _unpack_i16(data, 20),
            'stag_detected': _unpack_i16(data, 22),
            'arm_joint1': _unpack_i16(data, 24),
            'arm_joint2': _unpack_i16(data, 26),
            'arm_joint3': _unpack_i16(data, 28),
            'arm_yaw': _unpack_i16(data, 30),
            'x_offset': _unpack_i16(data, 32),
            'y_offset': _unpack_i16(data, 34),
        }
    except struct.error:
        return None


class Ack2BurstDetector(Node):
    def __init__(
        self,
        rx_topic: str,
        window_sec: float,
        threshold: int,
        context_before: int,
        context_after: int,
        cooldown_sec: float,
    ) -> None:
        super().__init__('ack2_burst_detector')

        self.window_sec = max(window_sec, 1e-6)
        self.threshold = max(threshold, 1)
        self.context_before = max(context_before, 0)
        self.context_after = max(context_after, 0)
        self.cooldown_sec = max(cooldown_sec, 0.0)

        self.ack2_times: Deque[float] = collections.deque()
        self.recent_packets: Deque[PacketView] = collections.deque(maxlen=max(self.context_before + 5, 16))

        self.last_trigger_time: float = -1e9
        self.pending_after_count = 0
        self.pending_event_header = ''

        self.sub = self.create_subscription(UInt8MultiArray, rx_topic, self._on_rx_msg, 50)

        self.get_logger().info(
            f'started: rx_topic={rx_topic}, window_sec={self.window_sec:.3f}, '
            f'threshold={self.threshold}, context_before={self.context_before}, context_after={self.context_after}, '
            f'cooldown_sec={self.cooldown_sec:.3f}'
        )

    def _on_rx_msg(self, msg: UInt8MultiArray) -> None:
        now = self.get_clock().now().nanoseconds / 1e9
        ts_text = _dt.datetime.now().strftime('%H:%M:%S.%f')[:-3]

        data = bytes(msg.data)
        decoded = _decode_rx_packet(data)

        header_ok = len(data) >= 2 and (data[0], data[1]) == HEADER
        footer_ok = len(data) >= 2 and (data[-2], data[-1]) == FOOTER

        if decoded is None:
            line = f'[{ts_text}][RX] malformed len={len(data)} header={header_ok} footer={footer_ok}'
            pkt = PacketView(now, ts_text, len(data), line, None)
            self.recent_packets.append(pkt)
            self._emit_pending_after(pkt)
            return

        ack = decoded['ack_flag']
        line = (
            f"[{ts_text}][RX] action={decoded['action']} ack={ack} "
            f"real=({decoded['x_real']},{decoded['y_real']}) "
            f"target=({decoded['x_target']},{decoded['y_target']}) "
            f"heading={decoded['target_heading']} stair={decoded['stair_direction']} "
            f"stag=({decoded['stag_x']},{decoded['stag_y']}) detected={decoded['stag_detected']} "
            f"arm=[{decoded['arm_joint1']},{decoded['arm_joint2']},{decoded['arm_joint3']},{decoded['arm_yaw']}] "
            f"offset=({decoded['x_offset']},{decoded['y_offset']}) "
            f"len={len(data)} header={'OK' if header_ok else 'BAD'} footer={'OK' if footer_ok else 'BAD'}"
        )
        pkt = PacketView(now, ts_text, len(data), line, ack)
        self.recent_packets.append(pkt)

        # 如果之前触发了事件，持续输出触发后的若干包
        self._emit_pending_after(pkt)

        # 统计 ack=2 的时间窗次数
        if ack == 2:
            self.ack2_times.append(now)
            while self.ack2_times and (now - self.ack2_times[0] > self.window_sec):
                self.ack2_times.popleft()

            if len(self.ack2_times) >= self.threshold and (now - self.last_trigger_time >= self.cooldown_sec):
                self._trigger_event(now)

    def _emit_pending_after(self, pkt: PacketView) -> None:
        if self.pending_after_count <= 0:
            return
        self.get_logger().info(f'[ACK2-BURST][AFTER] {pkt.line}')
        self.pending_after_count -= 1
        if self.pending_after_count == 0 and self.pending_event_header:
            self.get_logger().info(f'[ACK2-BURST] End of context for event: {self.pending_event_header}')
            self.pending_event_header = ''

    def _trigger_event(self, now: float) -> None:
        self.last_trigger_time = now
        count = len(self.ack2_times)
        start = self.ack2_times[0]
        span_ms = (now - start) * 1000.0

        event_header = (
            f'ack=2 burst detected: count={count} within {span_ms:.1f}ms '
            f'(window={self.window_sec * 1000.0:.0f}ms, threshold={self.threshold})'
        )
        self.pending_event_header = event_header

        self.get_logger().warn(f'[ACK2-BURST] {event_header}')

        # 输出触发前上下文
        before = list(self.recent_packets)[-self.context_before:]
        for p in before:
            self.get_logger().info(f'[ACK2-BURST][BEFORE] {p.line}')

        # 设置触发后上下文输出计数
        self.pending_after_count = self.context_after


def main() -> None:
    parser = argparse.ArgumentParser(description='Detect ack=2 bursts in RX packets.')
    parser.add_argument('--rx-topic', default='stm32/read', help='RX topic (default: stm32/read)')
    parser.add_argument('--window', type=float, default=0.2, help='Time window in seconds (default: 0.2)')
    parser.add_argument('--threshold', type=int, default=2, help='Minimum ack=2 count in window (default: 2)')
    parser.add_argument('--context-before', type=int, default=5, help='Packets before trigger to print (default: 5)')
    parser.add_argument('--context-after', type=int, default=5, help='Packets after trigger to print (default: 5)')
    parser.add_argument('--cooldown', type=float, default=0.5, help='Minimum interval between two trigger logs in seconds (default: 0.5)')
    args = parser.parse_args()

    rclpy.init()
    node = Ack2BurstDetector(
        rx_topic=args.rx_topic,
        window_sec=args.window,
        threshold=args.threshold,
        context_before=args.context_before,
        context_after=args.context_after,
        cooldown_sec=args.cooldown,
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
