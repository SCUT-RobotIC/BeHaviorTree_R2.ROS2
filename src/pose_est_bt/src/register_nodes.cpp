#include "behaviortree_ros2/plugins.hpp"
#include "pose_est_bt/pose_solve_node.hpp"

BT_REGISTER_ROS_NODES(factory, params)
{
  factory.registerNodeType<pose_est_bt::PoseSolveNode>("PoseSolve", params);
}
