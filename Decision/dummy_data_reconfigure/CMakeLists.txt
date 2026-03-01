cmake_minimum_required(VERSION 3.8)
project(dummy_data_reconfigure)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# 查找自定义消息包
get_filename_component(_repo_root "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
set(_msg_process_cmake_hint "${_repo_root}/msg_process/install/msg_process/share/msg_process/cmake")
if(NOT msg_process_DIR AND EXISTS "${_msg_process_cmake_hint}/msg_processConfig.cmake")
  set(msg_process_DIR "${_msg_process_cmake_hint}")
endif()
find_package(msg_process REQUIRED)

# 设置C++标准
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# cpp 可执行文件
add_executable(serial_config src/serial_config.cpp)
add_executable(autoaim_config src/autoaim_config.cpp)

# 包含目录
target_include_directories(serial_config PRIVATE
   ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_include_directories(autoaim_config PRIVATE
   ${CMAKE_CURRENT_SOURCE_DIR}/include
)

ament_target_dependencies(serial_config
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  msg_process
)

ament_target_dependencies(autoaim_config
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  msg_process
)

# 安装C++可执行文件
install(TARGETS
  serial_config
  autoaim_config
  DESTINATION lib/${PROJECT_NAME}
)

# 安装启动文件
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# 安装include头文件
install(DIRECTORY include/
  DESTINATION include/
)

# 导出依赖
ament_export_dependencies(
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  msg_process
)

# 导出包含目录
ament_export_include_directories(include)

ament_package()