#include "yolo_spearhead_see_bt/yolo_spearhead_see_node.hpp"

namespace yolo_spearhead_see_bt
{

bool YoloNode::setGoal(Goal& goal)
{
  auto need = getInput<bool>("need");
  if(!need)
  {
    return false;
  }

  goal.need = need.value();
  return true;
}

BT::PortsList YoloNode::providedPorts()
{
  return providedBasicPorts({
    BT::InputPort<bool>("need", true, "Whether to run YOLO detection"),
    BT::OutputPort<bool>("success")
  });
}

BT::NodeStatus YoloNode::onResultReceived(const WrappedResult& result)
{
  setOutput("success", result.result->success);
  return result.result->success ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace yolo_spearhead_see_bt