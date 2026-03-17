#pragma once

#include <string>
#include <vector>

#include "behaviortree_ros2/bt_service_node.hpp"
#include "foundationpose_interface/srv/pose_solve.hpp"

namespace pose_est_bt
{

class PoseSolveNode : public BT::RosServiceNode<foundationpose_interface::srv::PoseSolve>
{
public:
  PoseSolveNode(const std::string& name, const BT::NodeConfig& config,
                const BT::RosNodeParams& params)
  : BT::RosServiceNode<foundationpose_interface::srv::PoseSolve>(name, config, params)
  {
  }

  static BT::PortsList providedPorts();

  bool setRequest(Request::SharedPtr& request) override;
  BT::NodeStatus onResponseReceived(const Response::SharedPtr& response) override;
};

}  // namespace pose_est_bt
