cmake_minimum_required(VERSION 3.10)
project(robot_arm_bt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(pose_interface REQUIRED)
find_package(std_msgs REQUIRED)

add_library(robot_arm_bt_nodes SHARED
  src/robot_arm_publish_node.cpp
  src/robot_arm_solve_node.cpp
  src/register_nodes.cpp
)

target_include_directories(robot_arm_bt_nodes PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(robot_arm_bt_nodes
  rclcpp
  behaviortree_ros2
  pose_interface
  std_msgs
)

target_compile_definitions(robot_arm_bt_nodes PRIVATE BT_PLUGIN_EXPORT)

install(TARGETS robot_arm_bt_nodes
  DESTINATION share/${PROJECT_NAME}/lib
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()