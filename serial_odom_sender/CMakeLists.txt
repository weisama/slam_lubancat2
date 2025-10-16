cmake_minimum_required(VERSION 3.5)
project(serial_odom_sender)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(serial_odom_sender src/serial_odom_sender.cpp)

ament_target_dependencies(serial_odom_sender
  rclcpp
  nav_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

install(TARGETS
  serial_odom_sender
  DESTINATION lib/${PROJECT_NAME})

ament_package()

