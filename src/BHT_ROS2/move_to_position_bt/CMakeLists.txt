cmake_minimum_required(VERSION 3.8)
project(move_to_position_bt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(movetp_interface REQUIRED)

add_library(move_to_position_bt_nodes SHARED
  src/movetp_node.cpp
  src/register_nodes.cpp
)

ament_target_dependencies(move_to_position_bt_nodes
  rclcpp
  behaviortree_ros2
  movetp_interface
)

target_include_directories(move_to_position_bt_nodes PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_definitions(move_to_position_bt_nodes PRIVATE BT_PLUGIN_EXPORT)

install(TARGETS move_to_position_bt_nodes
  DESTINATION share/${PROJECT_NAME}/lib
)

install(DIRECTORY include/
  DESTINATION include
)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
