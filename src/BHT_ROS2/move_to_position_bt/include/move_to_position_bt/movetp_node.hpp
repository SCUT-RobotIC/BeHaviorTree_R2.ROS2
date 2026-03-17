#pragma once

#include <string>
#include <vector>

#include "behaviortree_ros2/bt_service_node.hpp"
#include "movetp_interface/srv/move_to_position.hpp"
    
namespace move_to_position_bt
{

class MoveToPositionNode : public BT::RosServiceNode<movetp_interface::srv::MoveToPosition>
{
public:
  MoveToPositionNode(const std::string& name, const BT::NodeConfig& config,
                     const BT::RosNodeParams& params)
  : BT::RosServiceNode<movetp_interface::srv::MoveToPosition>(name, config, params)
  {
  }

  static BT::PortsList providedPorts();

  bool setRequest(Request::SharedPtr& request) override;
  BT::NodeStatus onResponseReceived(const Response::SharedPtr& response) override;
};

}  // namespace move_to_position_bt
