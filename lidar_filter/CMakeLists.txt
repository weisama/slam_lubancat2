cmake_minimum_required(VERSION 3.8)
project(lidar_filter)

# 默认编译选项
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# 打印编译选项
add_compile_options(-Wall -Wextra -Wpedantic)

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# -------------------------------
# 构建可执行节点
# -------------------------------
add_executable(lidar_filter_node src/lidar_filter.cpp)
ament_target_dependencies(lidar_filter_node rclcpp sensor_msgs)

# 安装可执行文件
install(TARGETS
  lidar_filter_node
  DESTINATION lib/${PROJECT_NAME}
)

# -------------------------------
# 安装 launch 文件
# -------------------------------
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# -------------------------------
# 必需宏
# -------------------------------
ament_package()

