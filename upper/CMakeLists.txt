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

# ✅ 新增：nlohmann_json 库
find_package(nlohmann_json REQUIRED)

# 编译可执行文件
add_executable(upper src/upper.cpp)

ament_target_dependencies(upper
  rclcpp
  nav_msgs
  sensor_msgs
  tf2_msgs
  std_msgs
)

# ✅ 新增：链接 JSON 库
target_link_libraries(upper nlohmann_json::nlohmann_json)

# 安装节点
install(TARGETS upper
  DESTINATION lib/${PROJECT_NAME}
)

# 安装 launch 文件
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

