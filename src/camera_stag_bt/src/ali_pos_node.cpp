#include "camera_stag_bt/ali_ps_node.hpp"

namespace camera_stag_bt
{

bool AliPsNode::setGoal(Goal& goal)
{
  auto need = getInput<bool>("need");
  if(!need)
  {
    return false;
  }

  goal.need = need.value();
  return true;
}

BT::PortsList AliPsNode::providedPorts()
  {
    return providedBasicPorts({
      BT::InputPort<bool>("need", true, "Whether to run pose solve"),
      BT::OutputPort<bool>("success")
    });
  }

BT::NodeStatus AliPsNode::onResultReceived(const WrappedResult& result)
{
  setOutput("success", result.result->success);
  return result.result->success ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace camera_stag_bt
