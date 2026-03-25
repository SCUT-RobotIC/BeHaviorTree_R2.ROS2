#include "move_to_position_bt/movetp_node.hpp"

#include "geometry_msgs/msg/pose.hpp"

namespace move_to_position_bt
{


bool MoveToPositionNode::setGoal(Goal& goal)
{
  auto target_pose = getInput<geometry_msgs::msg::Pose>("target_position");
  if (!target_pose)
  {
    return false;
  }
  goal.goal = target_pose.value().position;
  return true;
}

BT::PortsList MoveToPositionNode::providedPorts()
  {
    return providedBasicPorts({
      BT::InputPort<geometry_msgs::msg::Pose>("target_position"),
    });
  }

BT::NodeStatus MoveToPositionNode::onResultReceived(const WrappedResult& result)
{
  return result.result->success ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace move_to_position_bt
