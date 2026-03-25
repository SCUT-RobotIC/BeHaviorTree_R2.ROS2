#pragma once

#include <string>
#include <vector>

#include "behaviortree_ros2/bt_action_node.hpp"
#include "rotation_interface/action/rotation.hpp"
    
namespace rotation_bt
{

class RotationNode : public BT::RosActionNode<rotation_interface::action::Rotation>
{
public:
  RotationNode(const std::string& name, const BT::NodeConfig& config,
               const BT::RosNodeParams& params)
    : BT::RosActionNode<rotation_interface::action::Rotation>(name, config, params)
  {
  }

  static BT::PortsList providedPorts();

  bool setGoal(Goal& goal) override;
  BT::NodeStatus onResultReceived(const WrappedResult& result) override;
};

}  // namespace rotation_bt