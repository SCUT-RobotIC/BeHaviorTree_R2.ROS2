#include "camera_stag_bt/ali_ps_node.hpp"

namespace camera_stag_bt
{

bool AliPsNode::setGoal(Goal& goal)
{
  goal.need = true;
  return true;
}

BT::PortsList AliPsNode::providedPorts()
  {
    BT::OutputPort<geometry_msgs::msg::Pose>("Position3"),
    return providedBasicPorts({
    });
  }

BT::NodeStatus AliPsNode::onResultReceived(const WrappedResult& result)
{
  geometry_msgs::msg::Pose Position3;
  Position3.position.x = 10;
  Position3.position.y = 20;
  Position3.position.z = 30;
  setOutput("Position3", Position3);
  return result.result->success ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace camera_stag_bt
