#pragma once

#include <string>
#include <vector>

#include "behaviortree_ros2/bt_action_node.hpp"
#include "fetchsp_interface/action/fetch_sp.hpp"

namespace yolo_spearhead_see_bt
{

class YoloNode : public BT::RosActionNode<fetchsp_interface::action::FetchSp>
{
public:
  YoloNode(const std::string& name, const BT::NodeConfig& config,
           const BT::RosNodeParams& params)
  : BT::RosActionNode<fetchsp_interface::action::FetchSp>(name, config, params)
  {
  }

  static BT::PortsList providedPorts();

  bool setGoal(Goal& goal) override;
  
  BT::NodeStatus onResultReceived(const WrappedResult& result) override;
};

}  // namespace yolo_spearhead_see_bt