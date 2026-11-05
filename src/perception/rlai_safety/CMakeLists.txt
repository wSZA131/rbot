cmake_minimum_required(VERSION 3.8)
project(rlai_safety)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rlai_interfaces REQUIRED)

add_executable(safety_monitor_node
  src/safety_monitor_node.cpp)

target_compile_features(safety_monitor_node PUBLIC cxx_std_17)

ament_target_dependencies(safety_monitor_node
  rclcpp
  sensor_msgs
  rlai_interfaces)

install(TARGETS safety_monitor_node
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
