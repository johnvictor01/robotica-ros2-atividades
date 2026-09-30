cmake_minimum_required(VERSION 3.8)
project(trajectory_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(control_toolbox REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(trajectory_controller src/trajectory_controller.cpp)
ament_target_dependencies(trajectory_controller
  control_toolbox geometry_msgs nav_msgs rclcpp std_msgs)

install(TARGETS trajectory_controller DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_package()
