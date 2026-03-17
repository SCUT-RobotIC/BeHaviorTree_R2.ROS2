cmake_minimum_required(VERSION 3.10)
project(r2_decision)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(fetchsp_interface REQUIRED)

install(DIRECTORY behavior_trees config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
