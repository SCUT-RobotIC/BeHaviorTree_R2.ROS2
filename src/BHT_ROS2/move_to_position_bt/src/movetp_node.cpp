#include "move_to_position_bt/movetp_node.hpp"

#include "geometry_msgs/msg/pose.hpp"

namespace move_to_position_bt
{

bool MoveToPositionNode::setRequest(Request::SharedPtr& request)
{
  auto target_pose = getInput<geometry_msgs::msg::Pose>("target_position");
  if (!target_pose)
  {
    return false;
  }

  request->target_position = target_pose.value();
  return true;
}

BT::PortsList MoveToPositionNode::providedPorts()
  {
    return providedBasicPorts({
      BT::InputPort<geometry_msgs::msg::Pose>("target_position"),
      BT::OutputPort<bool>("success")
    });
  }

BT::NodeStatus MoveToPositionNode::onResponseReceived(const Response::SharedPtr& response)
{
  setOutput("success", response->success);

  return response->success ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace move_to_position_bt
