cmake_minimum_required(VERSION 3.8)
project(tf)

# 检查编译器是否符合C++17标准
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

# 如果有C++节点，添加可执行文件
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/src/tf_broadcaster.cpp)
  add_executable(tf_broadcaster src/tf_broadcaster.cpp)
  ament_target_dependencies(tf_broadcaster 
    rclcpp 
    tf2_ros 
    geometry_msgs
  )
  
  # 安装可执行文件
  install(TARGETS
    tf_broadcaster
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

# 安装launch文件
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # 以下行会忽略ament_lint_auto找到的ament_lint_common包
  # 取消注释以执行lint测试
  # ament_lint_auto_find_test_dependencies()
endif()

ament_package()
