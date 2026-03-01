cmake_minimum_required(VERSION 3.10)
project(fastlio_relocator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

# 1. 查找 ROS 2 依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)

# 2. 查找 PCL 依赖
find_package(PCL REQUIRED)

# 3. 包含头文件目录
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

# 4. 添加可执行文件
add_executable(relocator_node src/relocator_node.cpp)

# 5. 链接 ROS 2 依赖
ament_target_dependencies(relocator_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2_ros
  pcl_conversions
)

# 6. 链接 PCL 依赖
target_link_libraries(relocator_node ${PCL_LIBRARIES})

# 7. 安装规则
install(TARGETS relocator_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

ament_package()