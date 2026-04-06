#pragma once

#include <string>
#include <vector>

#include "behaviortree_ros2/bt_service_node.hpp"
#include "pose_interface/srv/arm_solve.hpp"

namespace robot_arm_bt
{

class RobotArmSolveNode : public BT::RosServiceNode<pose_interface::srv::ArmSolve>
{
public:
  RobotArmSolveNode(const std::string& name, const BT::NodeConfig& config,
                    const BT::RosNodeParams& params)
  : BT::RosServiceNode<pose_interface::srv::ArmSolve>(name, config, params)
  {
  }

  static BT::PortsList providedPorts();

  bool setRequest(Request::SharedPtr& request) override;
  BT::NodeStatus onResponseReceived(const Response::SharedPtr& response) override;
};

}  // namespace robot_arm_bt