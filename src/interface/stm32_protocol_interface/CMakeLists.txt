cmake_minimum_required(VERSION 3.10)
project(stm32_protocol_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# 头文件安装
install(DIRECTORY include/
  DESTINATION include
)

# 导出包信息，使其他包能通过 find_package 找到
ament_export_include_directories(include)
ament_export_dependencies(rclcpp geometry_msgs tf2_ros tf2_geometry_msgs)

ament_package()
