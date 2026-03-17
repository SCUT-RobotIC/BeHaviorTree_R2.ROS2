#!/usr/bin/env python3
"""Monitor STM32 packets published to ROS2 topics."""

import argparse
import datetime as _dt
import struct
from typing import Dict, List, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray

HEADER = (0xAA, 0xAA)
FOOTER = (0xBB, 0xBB)
SEND_PACKET_SIZE = 64
RECV_PACKET_SIZE = 66


def _unpack_i16(data: bytes, offset: int) -> int:
    return struct.unpack_from('<h', data, offset)[0]


def _decode_tx_packet(data: bytes) -> Dict[str, int]:
    return {
        "action": _unpack_i16(data, 2),
        "x_real": _unpack_i16(data, 4),
        "y_real": _unpack_i16(data, 6),
        "x_target": _unpack_i16(data, 8),
        "y_target": _unpack_i16(data, 10),
        "target_heading": _unpack_i16(data, 12),
        "stair_direction": _unpack_i16(data, 14),
        "stag_x": _unpack_i16(data, 16),
        "stag_y": _unpack_i16(data, 18),
        "stag_detected": _unpack_i16(data, 20),
        "arm_joint1": _unpack_i16(data, 22),
        "arm_joint2": _unpack_i16(data, 24),
        "arm_joint3": _unpack_i16(data, 26),
        "arm_yaw": _unpack_i16(data, 28),
        "x_offset": _unpack_i16(data, 30),
        "y_offset": _unpack_i16(data, 32),
    }


def _decode_rx_packet(data: bytes) -> Dict[str, int]:
    return {
        "action": _unpack_i16(data, 2),
        "ack_flag": _unpack_i16(data, 4),
        "x_real": _unpack_i16(data, 6),
        "y_real": _unpack_i16(data, 8),
        "x_target": _unpack_i16(data, 10),
        "y_target": _unpack_i16(data, 12),
        "target_heading": _unpack_i16(data, 14),
        "stair_direction": _unpack_i16(data, 16),
        "stag_x": _unpack_i16(data, 18),
        "stag_y": _unpack_i16(data, 20),
        "stag_detected": _unpack_i16(data, 22),
        "arm_joint1": _unpack_i16(data, 24),
        "arm_joint2": _unpack_i16(data, 26),
        "arm_joint3": _unpack_i16(data, 28),
        "arm_yaw": _unpack_i16(data, 30),
        "x_offset": _unpack_i16(data, 32),
        "y_offset": _unpack_i16(data, 34),
    }


def _format_hex(data: bytes) -> str:
    return ' '.join(f"0x{b:02X}" for b in data)


def _validate_header_footer(data: bytes, footer_index: int) -> Tuple[bool, bool]:
    header_ok = len(data) >= 2 and (data[0], data[1]) == HEADER
    footer_ok = len(data) > footer_index + 1 and (data[footer_index], data[footer_index + 1]) == FOOTER
    return header_ok, footer_ok


class PacketMonitor(Node):
    def __init__(self, direction: str, tx_topic: str, rx_topic: str, print_hex: bool, print_reserved: bool):
        super().__init__('stm32_packet_monitor')
        self.print_hex = print_hex
        self.print_reserved = print_reserved
        self.direction = direction
        self.tx_topic = tx_topic
        self.rx_topic = rx_topic
        self.sub_tx = None
        self.sub_rx = None

        if direction in ("tx", "both"):
            self.sub_tx = self.create_subscription(UInt8MultiArray, tx_topic, self._on_tx_msg, 10)
        if direction in ("rx", "both"):
            self.sub_rx = self.create_subscription(UInt8MultiArray, rx_topic, self._on_rx_msg, 10)

    def _on_tx_msg(self, msg: UInt8MultiArray) -> None:
        self._handle_msg(msg, is_rx=False)

    def _on_rx_msg(self, msg: UInt8MultiArray) -> None:
        self._handle_msg(msg, is_rx=True)

    def _handle_msg(self, msg: UInt8MultiArray, is_rx: bool) -> None:
        data = bytes(msg.data)
        ts = _dt.datetime.now().strftime('%H:%M:%S.%f')[:-3]
        label = "RX" if is_rx else "TX"
        expected_size = RECV_PACKET_SIZE if is_rx else SEND_PACKET_SIZE
        footer_index = 64 if is_rx else 62

        if len(data) < expected_size:
            self.get_logger().warning(f"[{ts}][{label}] packet too short: {len(data)} bytes")
            if self.print_hex:
                self.get_logger().info(_format_hex(data))
            return

        header_ok, footer_ok = _validate_header_footer(data, footer_index)
        decoded = _decode_rx_packet(data) if is_rx else _decode_tx_packet(data)

        ack_str = f" ack={decoded['ack_flag']}" if is_rx else ""
        self.get_logger().info(
            f"[{ts}][{label}] action={decoded['action']}{ack_str} "
            f"real=({decoded['x_real']},{decoded['y_real']}) "
            f"target=({decoded['x_target']},{decoded['y_target']}) "
            f"heading={decoded['target_heading']} stair={decoded['stair_direction']} "
            f"stag=({decoded['stag_x']},{decoded['stag_y']}) detected={decoded['stag_detected']} "
            f"arm=[{decoded['arm_joint1']},{decoded['arm_joint2']},"
            f"{decoded['arm_joint3']},{decoded['arm_yaw']}] "
            f"offset=({decoded['x_offset']},{decoded['y_offset']}) "
            f"header={'OK' if header_ok else 'BAD'} footer={'OK' if footer_ok else 'BAD'}"
        )

        if self.print_reserved:
            if is_rx:
                reserved = list(data[36:64])
            else:
                reserved = list(data[34:62])
            self.get_logger().info(f"[{label}] reserved={reserved}")

        if self.print_hex:
            self.get_logger().info(_format_hex(data[:expected_size]))


def main() -> None:
    parser = argparse.ArgumentParser(description="Monitor STM32 packets via ROS2 topics.")
    parser.add_argument('--direction', choices=['tx', 'rx', 'both'], default='tx',
                        help='Select tx (send), rx (receive) or both (default: tx)')
    parser.add_argument('--tx-topic', default='stm32/write', help='TX topic to subscribe (default: stm32/write)')
    parser.add_argument('--rx-topic', default='stm32/read', help='RX topic to subscribe (default: stm32/read)')
    parser.add_argument('--topic', default=None, help='(deprecated) alias of --tx-topic')
    parser.add_argument('--print-hex', action='store_true', help='Print packet bytes in hex')
    parser.add_argument('--print-reserved', action='store_true', help='Print reserved bytes list')

    args = parser.parse_args()

    if args.topic:
        args.tx_topic = args.topic

    rclpy.init()
    node = PacketMonitor(args.direction, args.tx_topic, args.rx_topic, args.print_hex, args.print_reserved)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
