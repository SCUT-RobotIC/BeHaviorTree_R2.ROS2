#include "yolo_spearhead_see_bt/yolo_spearhead_see_node.hpp"

namespace yolo_spearhead_see_bt
{

bool YoloNode::setGoal(Goal& goal)
{
  goal.need = true;
  return true;
}

BT::PortsList YoloNode::providedPorts()
{
  return providedBasicPorts({
    BT::OutputPort<geometry_msgs::msg::Pose>("Align_Position", "The position to align with the target")
  }); 
}

BT::NodeStatus YoloNode::onResultReceived(const WrappedResult& result)
{
  geometry_msgs::msg::Pose target_position;
  target_position.position.x = 800;    
  target_position.position.y = 200;
  target_position.position.z = 0;
  target_position.orientation.x = 0.0;
  target_position.orientation.y = 0.0;  
  target_position.orientation.z = 0.0;
  target_position.orientation.w = 1.0;

  setOutput("Align_Position", target_position);
  
  return result.result->success ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace yolo_spearhead_see_bt