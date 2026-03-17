#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "stm32_comm/serial_manager.hpp"

class Stm32CommNode : public rclcpp::Node {
public:
    Stm32CommNode() : Node("stm32_comm_node") {
        this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
        this->declare_parameter<int>("baud_rate", 115200);

        std::string port = this->get_parameter("serial_port").as_string();
        int baud = this->get_parameter("baud_rate").as_int();

        serial_manager_ = std::make_unique<SerialManager>(this->get_logger());

        // Publisher for received data
        rx_pub_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>("stm32/read", 10);

        // Subscriber for data to send
        tx_sub_ = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
            "stm32/write", 10,
            std::bind(&Stm32CommNode::send_callback, this, std::placeholders::_1)
        );

        serial_manager_->set_receive_callback(
            std::bind(&Stm32CommNode::receive_callback, this, std::placeholders::_1)
        );

        if (serial_manager_->configure(port, baud)) {
            RCLCPP_INFO(this->get_logger(), "STM32 Comm Node initialized. Port: %s, Baud: %d", port.c_str(), baud);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to configure serial port.");
        }
    }

private:
    void send_callback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg) {
        if (!serial_manager_->send(msg->data)) {
            RCLCPP_WARN(this->get_logger(), "Failed to send data to STM32.");
        }
    }

    void receive_callback(const std::vector<uint8_t>& data) {
        auto msg = std_msgs::msg::UInt8MultiArray();
        msg.data = data;
        rx_pub_->publish(msg);
    }

    std::unique_ptr<SerialManager> serial_manager_;
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr rx_pub_;
    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr tx_sub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Stm32CommNode>());
    rclcpp::shutdown();
    return 0;
}
