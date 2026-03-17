#ifndef STM32_COMM__SERIAL_MANAGER_HPP_
#define STM32_COMM__SERIAL_MANAGER_HPP_

#include "stm32_comm/common_headers.hpp"

class SerialManager {
public:
    explicit SerialManager(rclcpp::Logger logger);
    ~SerialManager();

    SerialManager(const SerialManager&) = delete;
    SerialManager& operator=(const SerialManager&) = delete;

    bool configure(const std::string& port, int baudrate);
    bool send(const std::vector<uint8_t>& data);
    void set_receive_callback(std::function<void(const std::vector<uint8_t>&)> callback);

private:
    void read_loop();
    void process_buffer();

    rclcpp::Logger logger_;                 
    serial::Serial serial_port_;           
    std::thread read_thread_;               
    std::deque<uint8_t> receive_buffer_;    
    std::atomic<bool> running_;             
    
    std::function<void(const std::vector<uint8_t>&)> receive_callback_;
};

#endif // STM32_COMM__SERIAL_MANAGER_HPP_
