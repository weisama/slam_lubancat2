cmake_minimum_required(VERSION 3.5)
project(upper)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 添加 ROS2 路径到 CMAKE_PREFIX_PATH
list(APPEND CMAKE_PREFIX_PATH "/opt/ros/humble")

# -----------------------------------------------------------------------------
# 查找依赖
# -----------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# 使用 pkg-config 作为备用方案
find_package(PkgConfig REQUIRED)
pkg_check_modules(TINYXML2 tinyxml2)
if(TINYXML2_FOUND)
  message(STATUS "Found tinyxml2 via pkg-config")
else()
  # 如果 pkg-config 找不到，尝试手动设置
  find_library(TINYXML2_LIBRARIES NAMES tinyxml2)
  find_path(TINYXML2_INCLUDE_DIRS NAMES tinyxml2.h)
  if(TINYXML2_LIBRARIES AND TINYXML2_INCLUDE_DIRS)
    message(STATUS "Found tinyxml2 manually: ${TINYXML2_LIBRARIES}")
  else()
    message(FATAL_ERROR "Cannot find tinyxml2 library")
  endif()
endif()

# 查找 nlohmann_json
find_package(nlohmann_json REQUIRED)

# -----------------------------------------------------------------------------
# 编译节点
# -----------------------------------------------------------------------------
add_executable(${PROJECT_NAME}_node src/upper.cpp)

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  nav_msgs
  sensor_msgs
  tf2_msgs
  std_msgs
)

# 链接其他库
target_link_libraries(${PROJECT_NAME}_node
  ${nlohmann_json_LIBRARIES}
  ${TINYXML2_LIBRARIES}
)

# 添加包含目录
target_include_directories(${PROJECT_NAME}_node PRIVATE
  ${TINYXML2_INCLUDE_DIRS}
)

if(nlohmann_json_INCLUDE_DIRS)
  target_include_directories(${PROJECT_NAME}_node PRIVATE ${nlohmann_json_INCLUDE_DIRS})
endif()

# -----------------------------------------------------------------------------
# 安装
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

ament_package()
