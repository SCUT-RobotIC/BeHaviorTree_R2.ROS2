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
      BT::OutputPort<std::vector<double>>("theta_sum1"),
      BT::OutputPort<std::vector<double>>("theta_sum2"),
      BT::OutputPort<std::vector<double>>("theta_sum3"),
      BT::OutputPort<std::vector<double>>("theta_sum_yaw"),
      BT::OutputPort<bool>("success")
    });
  }

BT::NodeStatus PoseSolveNode::onResponseReceived(const Response::SharedPtr& response)
{
  setOutput("theta_sum1", response->theta_sum1);
  setOutput("theta_sum2", response->theta_sum2);
  setOutput("theta_sum3", response->theta_sum3);
  setOutput("theta_sum_yaw", response->theta_sum_yaw);
  setOutput("success", response->success);
  return response->success ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace pose_est_bt
