#include "pose_est_bt/pose_solve_node.hpp"

namespace pose_est_bt
{

bool PoseSolveNode::setRequest(Request::SharedPtr& request)
{
  auto need = getInput<bool>("need");
  if(!need)
  {
    return false;
  }

  request->need = need.value();
  return true;
}

BT::PortsList PoseSolveNode::providedPorts()
  {
    return providedBasicPorts({
      BT::InputPort<bool>("need", true, "Whether to run pose solve"),
      BT::OutputPort<double>("x"),
      BT::OutputPort<double>("y"),
      BT::OutputPort<double>("z"),
      BT::OutputPort<bool>("success")
    });
  }

BT::NodeStatus PoseSolveNode::onResponseReceived(const Response::SharedPtr& response)
{
  setOutput("x", static_cast<double>(response->x));
  setOutput("y", static_cast<double>(response->y));
  setOutput("z", static_cast<double>(response->z));
  setOutput("success", response->success);
  return response->success ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace pose_est_bt
