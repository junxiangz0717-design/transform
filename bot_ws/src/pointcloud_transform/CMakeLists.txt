cmake_minimum_required(VERSION 3.8)
project(pointcloud_transform)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

cmake_policy(SET CMP0074 NEW)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# 添加可执行文件
add_executable(transformer_node src/transformer_node.cpp)
ament_target_dependencies(transformer_node
  rclcpp
  sensor_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
  tf2_eigen
  geometry_msgs
  nav_msgs
  pcl_conversions
)

target_link_libraries(transformer_node ${PCL_LIBRARIES})

# 安装 launch 目录（后面会创建）
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# 安装可执行文件
install(TARGETS transformer_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
