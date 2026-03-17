#pragma once

#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_ros2/ros_node_params.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp/rclcpp.hpp"

namespace r2_initial
{

class R2InitialNode : public BT::StatefulActionNode
{
public:
  R2InitialNode(const std::string& name, const BT::NodeConfig& config,
                const BT::RosNodeParams& params);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  void createSubscription(const std::string& topic_name,
                          const std::string& topic_type,
                          std::size_t qos_depth,
                          const std::shared_ptr<std::atomic_bool>& received_flag);
  void resetSubscriptions();

  std::shared_ptr<rclcpp::Node> node_;
  std::vector<rclcpp::GenericSubscription::SharedPtr> subscriptions_;
  std::vector<std::shared_ptr<std::atomic_bool>> received_flags_;
  std::atomic<int> received_count_{0};
  std::size_t expected_count_{0};
  geometry_msgs::msg::Pose output_pose_;
  std::chrono::steady_clock::time_point start_time_;
};

}  // namespace r2_initial
