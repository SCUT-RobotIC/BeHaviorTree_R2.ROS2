cmake_minimum_required(VERSION 3.10)
project(stm32_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(foundationpose_interface REQUIRED)

include_directories(include)

add_executable(stm32_control_node
  src/stm32_control_node.cpp
  src/stm32_control_node_params.cpp
  src/stm32_control_node_runtime.cpp
  src/action_handler.cpp
  src/flow_registry.cpp
  src/flows.cpp
)

target_include_directories(stm32_control_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(stm32_control_node 
  rclcpp 
  geometry_msgs 
  std_msgs 
  tf2_ros 
  tf2_geometry_msgs
  nav2_msgs
  sensor_msgs
  foundationpose_interface
)

install(TARGETS stm32_control_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
