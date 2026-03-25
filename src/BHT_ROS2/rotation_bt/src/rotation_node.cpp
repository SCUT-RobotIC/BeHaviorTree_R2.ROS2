#include "rotation_bt/rotation_node.hpp"

#include "std_msgs/msg/int32.hpp"

namespace rotation_bt
{

bool RotationNode::setGoal(Goal& goal)
{
  auto target_direction = getInput<int>("target_direction");
  if (!target_direction)
  {
    return false;
  }
  goal.direction = target_direction.value();
  return true;
}


BT::PortsList RotationNode::providedPorts()
{
  return providedBasicPorts({
    BT::InputPort<int>("target_direction"),
  });
}


BT::NodeStatus RotationNode::onResultReceived(const WrappedResult& result)
{
  return result.result->success ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace rotation_bt
