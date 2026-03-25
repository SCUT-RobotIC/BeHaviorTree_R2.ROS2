#include "rotation_bt/rotation_node.hpp"

#include "std_msgs/msg/int32.hpp"

namespace rotation_bt
{

bool RotationNode::setGoal(Goal& goal)
{
  auto target_value = getInput<int>("target_value");
  if (!target_value)
  {
    return false;
  }
  goal.goal = target_value.value();
  return true;
}


BT::PortsList RotationNode::providedPorts()
{
  return providedBasicPorts({
    BT::InputPort<int>("target_value"),
  });
}


BT::NodeStatus RotationNode::onResultReceived(const WrappedResult& result)
{
  return result.result->success ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace rotation_bt
