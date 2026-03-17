#pragma once

#include <string>
#include <vector>

#include "behaviortree_ros2/bt_action_node.hpp"
#include "ali_interface/action/ali_sp.hpp"

namespace camera_stag_bt
{

class AliPsNode : public BT::RosActionNode<ali_interface::action::AliSp>
{
public:
  AliPsNode(const std::string& name, const BT::NodeConfig& config,
             const BT::RosNodeParams& params)
  : BT::RosActionNode<ali_interface::action::AliSp>(name, config, params)
  {
  }

  static BT::PortsList providedPorts();

  bool setGoal(Goal& goal) override;
  
  BT::NodeStatus onResultReceived(const WrappedResult& result) override;
};

}  // namespace camera_stag_bt