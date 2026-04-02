#include "stm32_comm/serial_manager.hpp"

/**
 * @brief configure() -> read_loop() -> process_buffer() -> receive_callback
 * @details configure() 打开串口并启动 read_loop 线程；read_loop 不断读取串口数据并追加到 receive_buffer_；process_buffer() 从 receive_buffer_中提取完整帧并调用 receive_callback_ 回调函数。
 * @note 通过 set_receive_callback() 注册回调函数，回调函数签名为 void(const std::vector<uint8_t>&)，接收完整帧数据。回调函数应尽量避免耗时操作或阻塞调用。
 * @warning 在回调中处理数据时应注意线程安全，避免访问 SerialManager 的成员变量（如 receive_buffer_）或其他共享资源；
 */

/**
 * @brief 构造函数，保存日志记录器并初始化运行标志
 * @param logger ROS2 日志记录器，由调用方传入（通常为 node->get_logger()）
 */
SerialManager::SerialManager(rclcpp::Logger logger)
: logger_(logger), running_(false) {}

/**
 * @brief 析构函数，停止读取线程并关闭串口
 *
 * 先将 running_ 置为 false，等待读取线程退出（join），
 * 再关闭串口（如果打开）。确保析构时不会有后台线程仍在访问成员。
 */
SerialManager::~SerialManager() {
    running_ = false; 
    if (read_thread_.joinable()) {
        read_thread_.join(); 
    }
    if (serial_port_.isOpen()) {
        serial_port_.close();
    }
}

bool SerialManager::configure(const std::string& port, int baudrate) {
    try {
        serial_port_.setPort(port);
        serial_port_.setBaudrate(baudrate);
        serial::Timeout timeout = serial::Timeout::simpleTimeout(100); 
        serial_port_.setTimeout(timeout);
        serial_port_.open();
        
        if (serial_port_.isOpen()) {
            RCLCPP_INFO(logger_, "Serial port '%s' opened successfully at %d baud.", port.c_str(), baudrate);
            // 启动后台读取线程
            running_ = true;
            read_thread_ = std::thread(&SerialManager::read_loop, this);
            return true;
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger_, "Failed to open serial port '%s': %s", port.c_str(), e.what());
    }
    return false;
}

bool SerialManager::send(const std::vector<uint8_t>& data) {
    if (!serial_port_.isOpen()) {
        RCLCPP_WARN(logger_, "Cannot send data, serial port is not open.");
        return false;
    }
    // 返回实际写入的字节数，应该等于 data.size() 才算成功
    size_t bytes_written = serial_port_.write(data);
    return bytes_written == data.size();
}

/**
 * @brief 设置接收回调，接收到完整帧时会调用此回调
 * @param callback 可调用对象，签名为 void(const std::vector<uint8_t>&)
 *
 * 按值接收并使用 std::move 赋给成员，允许传入临时 lambda 或 bind 结果。 std::move 把左值显示转换为右值引用
 */
void SerialManager::set_receive_callback(std::function<void(const std::vector<uint8_t>&)> callback) {
    receive_callback_ = std::move(callback);
}

void SerialManager::read_loop() {
    RCLCPP_INFO(logger_, "Serial reading thread started.");
    while (running_) {
        try {
            if (serial_port_.available() > 0) {
                std::vector<uint8_t> chunk;
                serial_port_.read(chunk, serial_port_.available());
                receive_buffer_.insert(receive_buffer_.end(), chunk.begin(), chunk.end());
                process_buffer();
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(logger_, "Error in serial read loop: %s", e.what());
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }
    RCLCPP_INFO(logger_, "Serial reading thread stopped.");
}

void SerialManager::process_buffer() {
    while (receive_buffer_.size() >= serial_protocol::PACKET_SIZE) {
        // 查找帧头位置
        auto header_it = std::search(
            receive_buffer_.begin(), receive_buffer_.end(),
            serial_protocol::HEADER.begin(), serial_protocol::HEADER.end());

        // 如果找不到帧头（帧头在缓冲区末尾），丢弃前面多余的字节（保留可能的部分帧头）
        if (header_it == receive_buffer_.end()) {
            size_t discard_count = receive_buffer_.size() - (serial_protocol::HEADER.size() - 1);
            if (discard_count > 0) {
                receive_buffer_.erase(receive_buffer_.begin(), receive_buffer_.begin() + discard_count);
            }
            return; 
        }

        // 如果帧头不在缓冲区开头，丢弃前面多余的字节
        if (header_it != receive_buffer_.begin()) {
            receive_buffer_.erase(receive_buffer_.begin(), header_it);
            continue; 
        }

        // 检查是否有足够的字节组成完整帧
        if (receive_buffer_.size() < serial_protocol::PACKET_SIZE) {
            return; 
        }

        std::vector<uint8_t> packet_data(receive_buffer_.begin(), receive_buffer_.begin() + serial_protocol::PACKET_SIZE);
        
        // 验证帧尾
        bool footer_match = std::equal(packet_data.end() - 2, packet_data.end(), serial_protocol::FOOTER.begin());

        if (footer_match) {
            // 调用回调：拷贝到局部变量以减小并发竞态（另一个线程可能正在修改成员回调）
            auto cb = receive_callback_;
            if (cb) {
                try {
                    cb(packet_data);
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(logger_, "Receive callback threw exception: %s", e.what());
                }
            }
            receive_buffer_.erase(receive_buffer_.begin(), receive_buffer_.begin() + serial_protocol::PACKET_SIZE);
        } else {
            receive_buffer_.erase(receive_buffer_.begin());
        }
    }
}
