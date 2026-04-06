#pragma once

#include <array>
#include <memory>
#include <vector>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_ros2/ros_node_params.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace robot_arm_bt
{

class RobotArmPublishNode : public BT::SyncActionNode
{
public:
  RobotArmPublishNode(const std::string& name, const BT::NodeConfig& config,
                      const BT::RosNodeParams& params);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  bool publishAngles(const std::array<double, 4>& angles);
  static std::array<double, 4> lastAnglesFromVectors(const std::vector<double>& theta_sum1,
                                                     const std::vector<double>& theta_sum2,
                                                     const std::vector<double>& theta_sum3,
                                                     const std::vector<double>& theta_sum_yaw);

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64MultiArray>> publisher_;
  bool published_ = false;
  std::array<double, 4> last_published_{{0.0, 0.0, 0.0, 0.0}};
};

}  // namespace robot_arm_bt