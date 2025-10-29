cmake_minimum_required(VERSION 3.5)
project(upper)

# -----------------------------------------------------------------------------
# 设置 C++ 标准
# -----------------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# -----------------------------------------------------------------------------
# 查找依赖包
# -----------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)   # ✅ JSON 支持

# -----------------------------------------------------------------------------
# 源码编译
# -----------------------------------------------------------------------------
add_executable(${PROJECT_NAME}_node src/upper.cpp)

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  nav_msgs
  sensor_msgs
  tf2_msgs
  std_msgs
)

# ✅ 链接 JSON 和 tinyxml2 库（系统库）
target_link_libraries(${PROJECT_NAME}_node
  nlohmann_json::nlohmann_json
  tinyxml2
)

# -----------------------------------------------------------------------------
# 安装目标与资源
# -----------------------------------------------------------------------------
install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

# -----------------------------------------------------------------------------
ament_package()

