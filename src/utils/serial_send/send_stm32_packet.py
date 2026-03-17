#!/usr/bin/env python3
"""Send a custom STM32 packet via ROS2 topic stm32/write.

Packet format follows stm32_control/include/stm32_control/serial_packet.hpp (SEND_PACKET_SIZE=64).
"""

import argparse
import struct
import time
from typing import Iterable, List

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray

HEADER = (0xAA, 0xAA)
FOOTER = (0xBB, 0xBB)
SEND_PACKET_SIZE = 64


def _parse_reserved(raw: str) -> List[int]:
    if not raw:
        return [0] * 32
    cleaned = raw.replace(" ", "").replace("0x", "").replace("0X", "")
    if len(cleaned) % 2 != 0:
        raise ValueError("reserved hex string length must be even")
    data = [int(cleaned[i:i + 2], 16) for i in range(0, len(cleaned), 2)]
    if len(data) > 32:
        raise ValueError("reserved bytes length must be <= 32")
    return data + [0] * (32 - len(data))


def _pack_i16(value: int) -> Iterable[int]:
    if value < -32768 or value > 32767:
        raise ValueError(f"int16 out of range: {value}")
    return struct.pack('<h', int(value))


def build_packet(args: argparse.Namespace) -> bytearray:
    buf = bytearray(SEND_PACKET_SIZE)
    buf[0], buf[1] = HEADER

    fields = [
        args.action,
        args.x_real,
        args.y_real,
        args.x_target,
        args.y_target,
        args.stag_x,
        args.stag_y,
        args.stag_detected,
        args.arm_joint1,
        args.arm_joint2,
        args.arm_joint3,
        args.arm_yaw,
        args.x_offset,
        args.y_offset,
    ]

    offset = 2
    for v in fields:
        packed = _pack_i16(v)
        buf[offset] = packed[0]
        buf[offset + 1] = packed[1]
        offset += 2

    reserved = _parse_reserved(args.reserved)
    buf[30:62] = bytes(reserved)

    buf[62], buf[63] = FOOTER
    return buf


class PacketSender(Node):
    def __init__(self, topic: str):
        super().__init__('stm32_packet_sender')
        self.pub = self.create_publisher(UInt8MultiArray, topic, 10)

    def send(self, data: bytearray):
        msg = UInt8MultiArray()
        msg.data = list(data)
        self.pub.publish(msg)


def main() -> None:
    parser = argparse.ArgumentParser(description="Send custom STM32 packet via ROS2 topic.")
    parser.add_argument('--topic', default='stm32/write', help='ROS2 topic to publish (default: stm32/write)')
    parser.add_argument('--repeat', type=int, default=99999999999999, help='Number of times to send (default: 1)')
    parser.add_argument('--rate', type=float, default=10.0, help='Send rate in Hz when repeat>1')
    parser.add_argument('--print-hex', action='store_true', help='Print packet bytes in hex')

    parser.add_argument('--action', type=int, default=0)
    parser.add_argument('--x-real', type=int, default=0)
    parser.add_argument('--y-real', type=int, default=0)
    parser.add_argument('--x-target', type=int, default=0)
    parser.add_argument('--y-target', type=int, default=0)
    parser.add_argument('--stag-x', type=int, default=0)
    parser.add_argument('--stag-y', type=int, default=0)
    parser.add_argument('--stag-detected', type=int, default=0)
    parser.add_argument('--arm-joint1', type=int, default=0)
    parser.add_argument('--arm-joint2', type=int, default=0)
    parser.add_argument('--arm-joint3', type=int, default=0)
    parser.add_argument('--arm-yaw', type=int, default=0)
    parser.add_argument('--x-offset', type=int, default=0)
    parser.add_argument('--y-offset', type=int, default=0)
    parser.add_argument('--reserved', type=str, default='', help='Reserved bytes as hex string (<=32 bytes)')

    args = parser.parse_args()

    packet = build_packet(args)
    if args.print_hex:
        hex_str = ' '.join(f"0x{b:02X}" for b in packet)
        print(hex_str)

    rclpy.init()
    node = PacketSender(args.topic)

    if args.repeat <= 1:
        node.send(packet)
    else:
        period = 1.0 / max(args.rate, 1e-6)
        for _ in range(args.repeat):
            node.send(packet)
            time.sleep(period)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
