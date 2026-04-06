#pragma once

#include <string>
#include <vector>

#include "behaviortree_ros2/bt_action_node.hpp"
#include "ali_interface/action/ali_sp.hpp"
#include "geometry_msgs/msg/pose.hpp"

namespace camera_stag_bt
{

class AliPsNode : public BT::RosActionNode<ali_interface::action::AliSp>
{
public:
  AliPsNode(const std::string& name, const BT::NodeConfig& config,
             const BT::RosNodeParams& params);

  static BT::PortsList providedPorts();

  bool setGoal(Goal& goal) override;
  
  BT::NodeStatus onResultReceived(const WrappedResult& result) override;

private:
  bool default_need_ = true;
  double default_position3_x_ = 10.0;
  double default_position3_y_ = 20.0;
  double default_position3_z_ = 30.0;
};

}  // namespace camera_stag_bt