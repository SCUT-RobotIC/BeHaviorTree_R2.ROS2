// ========================================
// 定义串口通信数据包结构体及其相关函数 (v2.0 协议 + Padding)
// ========================================
#ifndef STM32_CONTROL__SERIAL_PACKET_HPP_
#define STM32_CONTROL__SERIAL_PACKET_HPP_

#include "stm32_control/common_headers.hpp"
#include "stm32_control/action_codes.hpp"
#include "stm32_control/utils.hpp"
#include <sstream>
#include <iomanip>

namespace serial_protocol {

const std::vector<uint8_t> HEADER = {0xAA, 0xAA};
const std::vector<uint8_t> FOOTER = {0xBB, 0xBB};

const size_t SEND_PACKET_SIZE = 64;
const size_t RECV_PACKET_SIZE = 66;
const size_t PACKET_SIZE = RECV_PACKET_SIZE; 

struct SerialPacket {
  uint8_t head[2] = {0xAA, 0xAA};
  int16_t action = static_cast<int16_t>(ActionCode::IDLE);
  int16_t ack_flag = 0;
  int16_t x_real = 0;
  int16_t y_real = 0;
  int16_t x_target = 0;
  int16_t y_target = 0;
  int16_t target_heading = 0;  // 0:前, 1:左, 2:后, 3:右
  int16_t stair_direction = 0; // 0:上台阶, 1:下台阶
  int16_t stag_x = 0;
  int16_t stag_y = 0;
  int16_t stag_detected = 0;
  int16_t arm_send_count = 0;
  int16_t arm_joint1 = 0;
  int16_t arm_joint2 = 0; 
  int16_t arm_joint3 = 0; 
  int16_t arm_yaw = 0;
  int16_t x_offset = 0;
  int16_t y_offset = 0;
  uint8_t reserved[26] = {0};
  uint8_t tail[2] = {0xBB, 0xBB};

  SerialPacket() {
    head[0] = HEADER[0];
    head[1] = HEADER[1];
    tail[0] = FOOTER[0];
    tail[1] = FOOTER[1];
  }
};

inline void serialize_packet(const SerialPacket& packet, std::vector<uint8_t>& buffer) {
  buffer.resize(SEND_PACKET_SIZE);

  buffer[0] = packet.head[0];
  buffer[1] = packet.head[1];
  buffer[2] = packet.action & 0xFF;
  buffer[3] = (packet.action >> 8) & 0xFF;
  buffer[4] = packet.x_real & 0xFF;
  buffer[5] = (packet.x_real >> 8) & 0xFF;
  buffer[6] = packet.y_real & 0xFF;
  buffer[7] = (packet.y_real >> 8) & 0xFF;
  buffer[8] = packet.x_target & 0xFF;
  buffer[9] = (packet.x_target >> 8) & 0xFF;
  buffer[10] = packet.y_target & 0xFF;
  buffer[11] = (packet.y_target >> 8) & 0xFF;
  buffer[12] = packet.target_heading & 0xFF;
  buffer[13] = (packet.target_heading >> 8) & 0xFF;
  buffer[14] = packet.stair_direction & 0xFF;
  buffer[15] = (packet.stair_direction >> 8) & 0xFF;
  buffer[16] = packet.stag_x & 0xFF;
  buffer[17] = (packet.stag_x >> 8) & 0xFF;
  buffer[18] = packet.stag_y & 0xFF;
  buffer[19] = (packet.stag_y >> 8) & 0xFF;
  buffer[20] = packet.stag_detected & 0xFF;
  buffer[21] = (packet.stag_detected >> 8) & 0xFF;
  buffer[22] = packet.arm_send_count & 0xFF;
  buffer[23] = (packet.arm_send_count >> 8) & 0xFF;
  buffer[24] = packet.arm_joint1 & 0xFF;
  buffer[25] = (packet.arm_joint1 >> 8) & 0xFF;
  buffer[26] = packet.arm_joint2 & 0xFF;
  buffer[27] = (packet.arm_joint2 >> 8) & 0xFF;
  buffer[28] = packet.arm_joint3 & 0xFF;
  buffer[29] = (packet.arm_joint3 >> 8) & 0xFF;
  buffer[30] = packet.arm_yaw & 0xFF;
  buffer[31] = (packet.arm_yaw >> 8) & 0xFF;
  buffer[32] = packet.x_offset & 0xFF;
  buffer[33] = (packet.x_offset >> 8) & 0xFF;
  buffer[34] = packet.y_offset & 0xFF;
  buffer[35] = (packet.y_offset >> 8) & 0xFF;

  for (size_t i = 0; i < 26; ++i) {
    buffer[36 + i] = packet.reserved[i];
  }

  buffer[62] = packet.tail[0];
  buffer[63] = packet.tail[1];
}

inline SerialPacket deserialize_packet(const std::vector<uint8_t>& data_buffer) {
  if (data_buffer.size() < RECV_PACKET_SIZE) {
    throw std::runtime_error("Received data buffer is too small to deserialize a SerialPacket.");
  }

  SerialPacket packet;

  packet.head[0] = data_buffer[0];
  packet.head[1] = data_buffer[1];
  packet.action = utils::combine_bytes<int16_t>(&data_buffer[2]);
  packet.ack_flag = utils::combine_bytes<int16_t>(&data_buffer[4]);
  packet.x_real = utils::combine_bytes<int16_t>(&data_buffer[6]);
  packet.y_real = utils::combine_bytes<int16_t>(&data_buffer[8]);
  packet.x_target = utils::combine_bytes<int16_t>(&data_buffer[10]);
  packet.y_target = utils::combine_bytes<int16_t>(&data_buffer[12]);
  packet.target_heading = utils::combine_bytes<int16_t>(&data_buffer[14]);
  packet.stair_direction = utils::combine_bytes<int16_t>(&data_buffer[16]);
  packet.stag_x = utils::combine_bytes<int16_t>(&data_buffer[18]);
  packet.stag_y = utils::combine_bytes<int16_t>(&data_buffer[20]);
  packet.stag_detected = utils::combine_bytes<int16_t>(&data_buffer[22]);
  packet.arm_send_count = utils::combine_bytes<int16_t>(&data_buffer[24]);
  packet.arm_joint1 = utils::combine_bytes<int16_t>(&data_buffer[26]);
  packet.arm_joint2 = utils::combine_bytes<int16_t>(&data_buffer[28]);
  packet.arm_joint3 = utils::combine_bytes<int16_t>(&data_buffer[30]);
  packet.arm_yaw = utils::combine_bytes<int16_t>(&data_buffer[32]);
  packet.x_offset = utils::combine_bytes<int16_t>(&data_buffer[34]);
  packet.y_offset = utils::combine_bytes<int16_t>(&data_buffer[36]);

  for (size_t i = 0; i < 26; ++i) {
    packet.reserved[i] = data_buffer[38 + i];
  }

  packet.tail[0] = data_buffer[64];
  packet.tail[1] = data_buffer[65];

  return packet;
}

inline std::string format_packet_for_debug(const SerialPacket& packet, bool include_hex = true) {
  std::stringstream ss;
  ss << "Action: " << static_cast<int>(packet.action)
     << ", ACK: " << static_cast<int>(packet.ack_flag)
     << ", Real: (" << packet.x_real << "," << packet.y_real << ")"
     << ", Target: (" << packet.x_target << "," << packet.y_target << ")"
    << ", Heading: " << packet.target_heading
    << ", StairDir: " << packet.stair_direction
     << ", STAG: (" << packet.stag_x << "," << packet.stag_y << ")"
     << ", Detected: " << static_cast<int>(packet.stag_detected)
    << ", ArmSendCount: " << packet.arm_send_count
    << ", Arm: [" << packet.arm_joint1/10.0 << "," << packet.arm_joint2/10.0 
    << "," << packet.arm_joint3/10.0 << "," << packet.arm_yaw/10.0 << "]"
    << ", YOLO Offset: (" << packet.x_offset << "," << packet.y_offset << ")";

  if (include_hex) {
    ss << "\nHEX (Sender fmt): ";
    std::vector<uint8_t> buffer;
    serialize_packet(packet, buffer);
    for (const auto& byte : buffer) {
      ss << "0x" << std::setfill('0') << std::setw(2) << std::hex << static_cast<int>(byte) << " ";
    }
  }
  return ss.str();
}

} // namespace serial_protocol

#endif // STM32_CONTROL__SERIAL_PACKET_HPP_
