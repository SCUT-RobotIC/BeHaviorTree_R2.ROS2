#include "yolo_spearhead_see_bt/yolo_spearhead_see_node.hpp"

namespace yolo_spearhead_see_bt
{

bool YoloNode::setGoal(Goal& goal)
{
  auto target = getInput<std::string>("target");
  if(!target)
  {
    return false;
  }

  goal.target = target.value();
  return true;
}

BT::PortsList YoloNode::providedPorts()
{
  return providedBasicPorts({
    BT::InputPort<std::string>("need"),
    BT::OutputPort<bool>("detected")
  });
}

BT::NodeStatus YoloNode::onResultReceived(const WrappedResult& result)
{
  setOutput("detected", result.result->detected);
  return result.result->detected ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace yolo_spearhead_see_bt