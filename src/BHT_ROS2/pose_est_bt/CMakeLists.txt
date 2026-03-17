cmake_minimum_required(VERSION 3.10)
project(pose_est_bt)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(foundationpose_interface REQUIRED)

add_library(pose_est_bt_nodes SHARED
  src/pose_solve_node.cpp
  src/register_nodes.cpp
)

target_include_directories(pose_est_bt_nodes PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(pose_est_bt_nodes
  rclcpp
  behaviortree_ros2
  foundationpose_interface
)

target_compile_definitions(pose_est_bt_nodes PRIVATE BT_PLUGIN_EXPORT)

install(TARGETS pose_est_bt_nodes
  DESTINATION share/${PROJECT_NAME}/lib
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
