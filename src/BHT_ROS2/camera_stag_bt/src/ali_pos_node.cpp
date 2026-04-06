#include "camera_stag_bt/ali_ps_node.hpp"

namespace camera_stag_bt
{

AliPsNode::AliPsNode(const std::string& name, const BT::NodeConfig& config,
                     const BT::RosNodeParams& params)
  : BT::RosActionNode<ali_interface::action::AliSp>(name, config, params)
{
  if (auto node = node_.lock()) {
    node->declare_parameter<bool>("camera_stag_need", true);
    node->declare_parameter<double>("", 10.0);
    node->declare_parameter<double>("position3_y", 20.0);
    node->declare_parameter<double>("position3_z", 30.0);

    node->get_parameter("camera_stag_need", default_need_);
    node->get_parameter("position3_x", default_position3_x_);
    node->get_parameter("position3_y", default_position3_y_);
    node->get_parameter("position3_z", default_position3_z_);
  }
}

bool AliPsNode::setGoal(Goal& goal)
{
  bool need = default_need_;
  if (auto input_need = getInput<bool>("need")) {
    need = input_need.value();
  }
  goal.need = need;
  return true;
}

BT::PortsList AliPsNode::providedPorts()
  {
    return providedBasicPorts({
      BT::InputPort<bool>("need", "Whether to trigger AliSp action"),
      BT::OutputPort<geometry_msgs::msg::Pose>("Position3")
    });
  }

BT::NodeStatus AliPsNode::onResultReceived(const WrappedResult& result)
{
  geometry_msgs::msg::Pose Position3;
  Position3.position.x = default_position3_x_;
  Position3.position.y = default_position3_y_;
  Position3.position.z = default_position3_z_;
  setOutput("Position3", Position3);
  return result.result->success ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace camera_stag_bt
