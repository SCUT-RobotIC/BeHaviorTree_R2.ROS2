cmake_minimum_required(VERSION 3.10)
project(camera_stag_bt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(ali_interface REQUIRED)

add_library(camera_stag_bt_nodes SHARED
  src/ali_pos_node.cpp
  src/register_nodes.cpp
)

target_include_directories(camera_stag_bt_nodes PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(camera_stag_bt_nodes
  rclcpp
  behaviortree_ros2
  ali_interface
)

target_compile_definitions(camera_stag_bt_nodes PRIVATE BT_PLUGIN_EXPORT)

install(TARGETS camera_stag_bt_nodes
  DESTINATION share/${PROJECT_NAME}/lib
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()