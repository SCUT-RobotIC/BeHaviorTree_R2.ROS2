#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "stm32_comm/serial_manager.hpp"

/**
 * @file stm32_comm_node.cpp
 * @brief 串口通信节点：将串口层与 ROS 主题连接
 */

class Stm32CommNode : public rclcpp::Node {
public:
    /**
     * @brief 构造函数：声明参数、初始化 SerialManager、设置 ROS 发布/订阅
     */
    Stm32CommNode() : Node("stm32_comm_node") {
        this->declare_parameter<std::string>("serial_port", "/dev/ttyACM0");
        this->declare_parameter<int>("baud_rate", 115200);

        std::string port = this->get_parameter("serial_port").as_string();
        int baud = this->get_parameter("baud_rate").as_int();

        // 创建 SerialManager 实例（负责串口 I/O）
        serial_manager_ = std::make_unique<SerialManager>(this->get_logger());

        // 接收数据的发布者：将接收到的字节数组发布到主题 stm32/read
        rx_pub_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>("stm32/read", 10);

        // 发送数据的订阅者：从主题 stm32/write 接收待发送的数据
        tx_sub_ = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
            "stm32/write", 10,
            std::bind(&Stm32CommNode::send_callback, this, std::placeholders::_1)
        );

        // 将节点的成员函数绑定为串口接收回调（注意对象生命周期）
        serial_manager_->set_receive_callback(
            std::bind(&Stm32CommNode::receive_callback, this, std::placeholders::_1)
        );

        // 配置并打开串口，成功后启动后台读取线程
        if (serial_manager_->configure(port, baud)) {
            RCLCPP_INFO(this->get_logger(), "STM32 Comm Node initialized. Port: %s, Baud: %d", port.c_str(), baud);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to configure serial port.");
        }
    }

    // 析构函数：先释放 SerialManager，确保后台读取线程停止，
    // 避免在发布者/订阅者被销毁后仍有串口线程调用回调。
    ~Stm32CommNode() {
        serial_manager_.reset();
    }

private:
    void send_callback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg) {
        if (!serial_manager_->send(msg->data)) {
            RCLCPP_WARN(this->get_logger(), "Failed to send data to STM32.");
        }
    }

    void receive_callback(const std::vector<uint8_t>& data) {
        // 将接收到的字节包封装为 ROS 消息并发布
        auto msg = std_msgs::msg::UInt8MultiArray();
        msg.data = data;
        rx_pub_->publish(msg);
    }

    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr rx_pub_;
    std::unique_ptr<SerialManager> serial_manager_;
    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr tx_sub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Stm32CommNode>());
    rclcpp::shutdown();
    return 0;
}
