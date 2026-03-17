cmake_minimum_required(VERSION 3.16)
project(glim2small_gicp LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(small_gicp REQUIRED)

add_executable(glim2small_gicp_node
  src/glim2small_gicp_node.cpp
)

target_include_directories(glim2small_gicp_node PRIVATE
  ${PCL_INCLUDE_DIRS}
)

target_link_libraries(glim2small_gicp_node
  Eigen3::Eigen
  small_gicp::small_gicp
  ${PCL_LIBRARIES}
)

ament_target_dependencies(glim2small_gicp_node
  rclcpp
  sensor_msgs
  geometry_msgs
)

install(TARGETS glim2small_gicp_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()