#ifndef STM32_COMM__SERIAL_MANAGER_HPP_
#define STM32_COMM__SERIAL_MANAGER_HPP_

#include "stm32_comm/common_headers.hpp"
#include "stm32_protocol_interface/serial_packet.hpp"

/**
 * @class SerialManager
 * @brief 串口收发类，支持异步读写和回调通知
 * @details 封装底层串口操作，提供线程安全的读写接口
 *
 * 封装串口打开/配置、数据发送以及后台读取逻辑。读取线程将收到的
 * 原始字节追加到内部缓冲区，由内部解析器提取完整数据包并通过
 * 用户注册的回调交付上层。
 *
 * @note 通过 `configure()` 打开串口并启动后台读取线程；析构或调用
 *       停止逻辑前应确保读取线程已退出以避免竞态条件。
 * @warning 在回调中应避免耗时操作或阻塞调用；如需长时间处理，请
 *          将数据转发到另一个工作线程或队列中。
 *
 * @author LiangShuang
 * @date 2026-03-31
 * @version 1.0.0
 */

class SerialManager {
public:
    explicit SerialManager(rclcpp::Logger logger);
    ~SerialManager();

    // 不允许复制和赋值
    SerialManager(const SerialManager&) = delete;
    SerialManager& operator=(const SerialManager&) = delete;

    /**
     * @brief 配置串口端口和波特率并打开串口
     * @param port 串口设备路径，例如 /dev/ttyUSB0
     * @param baudrate 波特率，例如 115200
     * @return 成功返回 true，失败返回 false
     */
    bool configure(const std::string& port, int baudrate);

    /**
     * @brief 发送数据到串口
     * @param data 待发送的字节序列
     * @return 发送成功返回 true，失败返回 false
     */
    bool send(const std::vector<uint8_t>& data);

    /**
     * @brief 设置接收回调，收到完整数据后会以字节向量形式回调
     * @param callback 回调函数，签名为 void(const std::vector<uint8_t>&)
     */
    void set_receive_callback(std::function<void(const std::vector<uint8_t>&)> callback);

private:
    /**
     * @brief 后台读取循环，运行在独立线程中
     */
    void read_loop();

    /**
     * @brief 处理内部接收缓冲区，提取完整帧并触发回调
     */
    void process_buffer();

    rclcpp::Logger logger_;                 ///< ROS2 日志记录器
    serial::Serial serial_port_;           ///< 串口对象
    std::thread read_thread_;               ///< 串口读取线程
    std::deque<uint8_t> receive_buffer_;    ///< 接收字节缓冲区
    std::atomic<bool> running_;             ///< 控制读取线程的运行状态

    std::function<void(const std::vector<uint8_t>&)> receive_callback_; ///< 用户接收回调
};

#endif // STM32_COMM__SERIAL_MANAGER_HPP_
