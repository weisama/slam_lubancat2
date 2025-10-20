cmake_minimum_required(VERSION 3.5)
project(upper)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 找 ROS 2 依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# 编译可执行文件
add_executable(serial_sender src/upper.cpp)

ament_target_dependencies(serial_sender
  rclcpp
  nav_msgs
  sensor_msgs
  tf2_msgs
  std_msgs
)

# 安装节点
install(TARGETS serial_sender
  DESTINATION lib/${PROJECT_NAME}
)

# 安装 launch 文件
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()