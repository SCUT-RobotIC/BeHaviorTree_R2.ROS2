#include "robot_arm_bt/robot_arm_solve_node.hpp"

namespace robot_arm_bt
{

bool RobotArmSolveNode::setRequest(Request::SharedPtr& request)
{
  auto x = getInput<double>("x");
  auto y = getInput<double>("y");
  auto z = getInput<double>("z");
  if(!x || !y || !z)
  {
    return false;
  }

  request->x = x.value();
  request->y = y.value();
  request->z = z.value();
  return true;
}

BT::PortsList RobotArmSolveNode::providedPorts()
{
  return providedBasicPorts({
    BT::InputPort<double>("x"),
    BT::InputPort<double>("y"),
    BT::InputPort<double>("z"),
    BT::OutputPort<std::vector<double>>("theta_sum1"),
    BT::OutputPort<std::vector<double>>("theta_sum2"),
    BT::OutputPort<std::vector<double>>("theta_sum3"),
    BT::OutputPort<std::vector<double>>("theta_sum_yaw"),
    BT::OutputPort<bool>("success")
  });
}

BT::NodeStatus RobotArmSolveNode::onResponseReceived(const Response::SharedPtr& response)
{
  setOutput("theta_sum1", response->theta_sum1);
  setOutput("theta_sum2", response->theta_sum2);
  setOutput("theta_sum3", response->theta_sum3);
  setOutput("theta_sum_yaw", response->theta_sum_yaw);
  setOutput("success", response->success);
  return response->success ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace robot_arm_bt