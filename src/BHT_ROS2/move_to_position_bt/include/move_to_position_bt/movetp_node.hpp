#pragma once

#include <string>
#include <vector>

#include "behaviortree_ros2/bt_action_node.hpp"
#include "movetp_interface/action/move_to_position.hpp"
    
namespace move_to_position_bt
{

class MoveToPositionNode : public BT::RosActionNode<movetp_interface::action::MoveToPosition>
{
public:
  MoveToPositionNode(const std::string& name, const BT::NodeConfig& config,
                     const BT::RosNodeParams& params)
  : BT::RosActionNode<movetp_interface::action::MoveToPosition>(name, config, params)
  {
  }

  static BT::PortsList providedPorts();

  bool setGoal(Goal& goal) override;
  BT::NodeStatus onResultReceived(const WrappedResult& result) override;
};

}  // namespace move_to_position_bt
