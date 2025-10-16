cmake_minimum_required(VERSION 3.8)
project(gmapping_launch)

# 查找依赖
find_package(ament_cmake REQUIRED)

# 安装launch和config文件
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()