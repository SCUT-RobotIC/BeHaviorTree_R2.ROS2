cmake_minimum_required(VERSION 3.8)
project(r2_initial_bt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(geometry_msgs REQUIRED)

add_library(r2_initial_bt_nodes SHARED
  src/r2_initial_nodes.cpp
  src/register_nodes.cpp
)

target_include_directories(r2_initial_bt_nodes PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(r2_initial_bt_nodes
  rclcpp
  behaviortree_ros2
  geometry_msgs
)

target_compile_definitions(r2_initial_bt_nodes PRIVATE BT_PLUGIN_EXPORT)

install(TARGETS r2_initial_bt_nodes
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
