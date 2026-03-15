#pragma once

#include <string>
#include <vector>

#include "behaviortree_ros2/bt_action_node.hpp"
#include "yolo_spearhead_see/yolo_spearhead_see.hpp"

namespace yolo_spearhead_see_bt
{

class YoloNode : public BT::RosActionNode<yolo_spearhead_see::YoloAction>
{
public:
  YoloNode(const std::string& name, const BT::NodeConfig& config,
           const BT::RosNodeParams& params)
  : BT::RosActionNode<yolo_spearhead_see::YoloAction>(name, config, params)
  {
  }

  static BT::PortsList providedPorts();

  bool setGoal(Goal& goal) override;
  
  BT::NodeStatus onResultReceived(const WrappedResult& result) override;
};

}  // namespace yolo_spearhead_see_bt