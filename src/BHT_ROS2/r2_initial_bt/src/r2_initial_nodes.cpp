#include "r2_initial/subscribe_once_node.hpp"

#include <stdexcept>

namespace r2_initial
{
R2InitialNode::R2InitialNode(const std::string& name, const BT::NodeConfig& config,
                             const BT::RosNodeParams& params) : BT::StatefulActionNode(name, config)
{
  node_ = params.nh.lock();
  if (!node_)
  {
    throw std::runtime_error("R2InitialNode requires a valid rclcpp::Node");
  }
}

BT::PortsList R2InitialNode::providedPorts()
{
  return {
    BT::InputPort<double>("timeout", 2.0, "Timeout seconds"),
    BT::OutputPort<geometry_msgs::msg::Pose>("Position1")
  };
}

BT::NodeStatus R2InitialNode::onStart()
{
  received_count_.store(0);
  expected_count_ = 4;
  received_flags_.clear();
  received_flags_.reserve(expected_count_);
  start_time_ = std::chrono::steady_clock::now();

  resetSubscriptions();
  subscriptions_.reserve(expected_count_);

  for (std::size_t index = 0; index < expected_count_; ++index)
  {
    received_flags_.push_back(std::make_shared<std::atomic_bool>(false));
  }

  createSubscription("/Odometry", "nav_msgs.msg.Odometry", 10, received_flags_[0]);
  createSubscription("/camera/camera/color/image_raw", "sensor_msgs.msg.Image", 10, received_flags_[1]);
  createSubscription("/camera/camera/aligned_depth_to_color/image_raw", "sensor_msgs.msg.Image", 10, received_flags_[2]);
  createSubscription("/terraced_camera_image", "sensor_msgs.msg.Image", 10, received_flags_[3]);

  output_pose_ = geometry_msgs::msg::Pose();
  output_pose_.position.x = 0.0;    
  output_pose_.position.y = 0.0;
  output_pose_.position.z = 0.0;
  output_pose_.orientation.w = 1.0;

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus R2InitialNode::onRunning()
{
  if (expected_count_ > 0 &&
      static_cast<std::size_t>(received_count_.load()) >= expected_count_)
  {
    resetSubscriptions();
    setOutput("Position1", output_pose_);
    return BT::NodeStatus::SUCCESS;
  }

  auto timeout = getInput<double>("timeout");
  if (!timeout)
  {
    return BT::NodeStatus::FAILURE;
  }

  if (timeout.value() > 0.0)
  {
    const auto elapsed = std::chrono::steady_clock::now() - start_time_;
    if (elapsed > std::chrono::duration<double>(timeout.value()))
    {
      resetSubscriptions();
      setOutput("Position1", output_pose_);
      return BT::NodeStatus::FAILURE;
    }
  }

  return BT::NodeStatus::RUNNING;
}

void R2InitialNode::onHalted()
{
  resetSubscriptions();
  received_count_.store(0);
  expected_count_ = 0;
  received_flags_.clear();
}

void R2InitialNode::createSubscription(const std::string& topic_name,
                                       const std::string& topic_type,
                                       std::size_t qos_depth,
                                       const std::shared_ptr<std::atomic_bool>& received_flag)
{
  auto qos = rclcpp::QoS(rclcpp::KeepLast(static_cast<int>(qos_depth)));
  auto subscription = node_->create_generic_subscription(
    topic_name, topic_type, qos,
    [this, received_flag](std::shared_ptr<rclcpp::SerializedMessage>)
    {
      if (!received_flag->exchange(true))
      {
        received_count_.fetch_add(1);
      }
    });

  subscriptions_.push_back(subscription);
}

void R2InitialNode::resetSubscriptions()
{
  subscriptions_.clear();
}

}  // namespace r2_initial
