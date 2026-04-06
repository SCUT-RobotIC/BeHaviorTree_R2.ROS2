#include "robot_arm_bt/robot_arm_publish_node.hpp"

#include <stdexcept>

namespace robot_arm_bt
{

RobotArmPublishNode::RobotArmPublishNode(const std::string& name, const BT::NodeConfig& config,
                                         const BT::RosNodeParams& params)
: BT::SyncActionNode(name, config)
, node_(params.nh.lock())
{
  if(!node_)
  {
    throw BT::RuntimeError("RobotArmPublishNode: ROS node expired");
  }

  publisher_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>("/arm_angles", 1);
}

BT::PortsList RobotArmPublishNode::providedPorts()
{
  return {
    BT::InputPort<std::vector<double>>("theta_sum1"),
    BT::InputPort<std::vector<double>>("theta_sum2"),
    BT::InputPort<std::vector<double>>("theta_sum3"),
    BT::InputPort<std::vector<double>>("theta_sum_yaw"),
    BT::InputPort<bool>("success")
  };
}

std::array<double, 4> RobotArmPublishNode::lastAnglesFromVectors(
  const std::vector<double>& theta_sum1,
  const std::vector<double>& theta_sum2,
  const std::vector<double>& theta_sum3,
  const std::vector<double>& theta_sum_yaw)
{
  if(theta_sum1.empty() || theta_sum2.empty() || theta_sum3.empty() || theta_sum_yaw.empty())
  {
    throw BT::RuntimeError("RobotArmPublishNode: angle vector is empty");
  }

  return {theta_sum1.back(), theta_sum2.back(), theta_sum3.back(), theta_sum_yaw.back()};
}

bool RobotArmPublishNode::publishAngles(const std::array<double, 4>& angles)
{
  if(published_ && angles == last_published_)
  {
    return true;
  }

  std_msgs::msg::Float64MultiArray msg;
  msg.data = {angles[0], angles[1], angles[2], angles[3]};
  publisher_->publish(msg);
  last_published_ = angles;
  published_ = true;
  return true;
}

BT::NodeStatus RobotArmPublishNode::tick()
{
  auto success = getInput<bool>("success");
  auto theta_sum1 = getInput<std::vector<double>>("theta_sum1");
  auto theta_sum2 = getInput<std::vector<double>>("theta_sum2");
  auto theta_sum3 = getInput<std::vector<double>>("theta_sum3");
  auto theta_sum_yaw = getInput<std::vector<double>>("theta_sum_yaw");

  if(!success || !theta_sum1 || !theta_sum2 || !theta_sum3 || !theta_sum_yaw)
  {
    return BT::NodeStatus::FAILURE;
  }

  if(!success.value())
  {
    return BT::NodeStatus::FAILURE;
  }

  const auto angles = lastAnglesFromVectors(theta_sum1.value(), theta_sum2.value(),
                                            theta_sum3.value(), theta_sum_yaw.value());
  publishAngles(angles);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace robot_arm_bt