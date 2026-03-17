#include "stm32_comm/serial_manager.hpp"
#include "stm32_comm/serial_packet.hpp" 

SerialManager::SerialManager(rclcpp::Logger logger)
: logger_(logger), running_(false) {}

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
    size_t bytes_written = serial_port_.write(data);
    return bytes_written == data.size();
}

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
        auto header_it = std::search(
            receive_buffer_.begin(), receive_buffer_.end(),
            serial_protocol::HEADER.begin(), serial_protocol::HEADER.end());

        if (header_it == receive_buffer_.end()) {
            size_t discard_count = receive_buffer_.size() > (serial_protocol::HEADER.size() - 1) ? 
                                   receive_buffer_.size() - (serial_protocol::HEADER.size() - 1) : 0;
            if (discard_count > 0) {
                receive_buffer_.erase(receive_buffer_.begin(), receive_buffer_.begin() + discard_count);
            }
            return; 
        }

        if (header_it != receive_buffer_.begin()) {
            receive_buffer_.erase(receive_buffer_.begin(), header_it);
            continue; 
        }

        if (receive_buffer_.size() < serial_protocol::PACKET_SIZE) {
            return; 
        }

        std::vector<uint8_t> packet_data(receive_buffer_.begin(), receive_buffer_.begin() + serial_protocol::PACKET_SIZE);
        
        bool footer_match = std::equal(packet_data.end() - 2, packet_data.end(), serial_protocol::FOOTER.begin());

        if (footer_match) {
            if (receive_callback_) {
                receive_callback_(packet_data);
            }
            receive_buffer_.erase(receive_buffer_.begin(), receive_buffer_.begin() + serial_protocol::PACKET_SIZE);
        } else {
            receive_buffer_.erase(receive_buffer_.begin());
        }
    }
}
