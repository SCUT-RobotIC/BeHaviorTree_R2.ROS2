 #pragma once

 #include <memory>
 #include <string>

 #include "behaviortree_cpp/action_node.h"
 #include "behaviortree_ros2/ros_node_params.hpp"
 #include "geometry_msgs/msg/transform_stamped.hpp"
 #include "tf2_ros/buffer.h"
 #include "tf2_ros/transform_listener.h"

 namespace tf_listen_bt
 {

 class TfListenNode : public BT::SyncActionNode
 {
 public:
	 TfListenNode(const std::string& name, const BT::NodeConfig& config,
								const BT::RosNodeParams& params);

	 static BT::PortsList providedPorts();

	 BT::NodeStatus tick() override;

 private:
	 std::shared_ptr<rclcpp::Node> node_;
	 std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
	 std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
	 std::double error = 0.1; // 可调整的误差范围
 };

 }  // namespace tf_listen_bt
