cmake_minimum_required(VERSION 3.8)
project(fast_lio)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# -------------------------------------------------------
# 1. 设置编译标准与选项
# -------------------------------------------------------
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_compile_options(-O3 -pthread -fexceptions -w -Wfatal-errors)
add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

message("Current CPU archtecture: ${CMAKE_SYSTEM_PROCESSOR}")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)" )
  include(ProcessorCount)
  ProcessorCount(N)
  message("Processer number:  ${N}")
  if(N GREATER 4)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=3)
    message("core for MP: 3")
  elseif(N GREATER 3)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=2)
    message("core for MP: 2")
  else()
    add_definitions(-DMP_PROC_NUM=1)
  endif()
else()
  add_definitions(-DMP_PROC_NUM=1)
endif()

find_package(OpenMP QUIET)
if(OpenMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

# -------------------------------------------------------
# 2. 寻找依赖包
# -------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(livox_ros_driver2 REQUIRED) 
find_package(rosidl_default_generators REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

# [新增] OpenCV 用于 FFT 重定位
find_package(OpenCV REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Development)

# -------------------------------------------------------
# 3. 包含目录
# -------------------------------------------------------
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${Python3_INCLUDE_DIRS}
  # [新增] OpenCV 头文件
  ${OpenCV_INCLUDE_DIRS}
)

# -------------------------------------------------------
# 4. 生成自定义消息
# -------------------------------------------------------
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Pose6D.msg"
  DEPENDENCIES geometry_msgs
)

# -------------------------------------------------------
# 5. 生成可执行文件
# -------------------------------------------------------
add_executable(fastlio_mapping 
  src/laserMapping.cpp 
  src/preprocess.cpp
  include/ikd-Tree/ikd_Tree.cpp
  include/voxel_map/voxel_map.cpp
)

ament_target_dependencies(fastlio_mapping
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  pcl_conversions
  tf2
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  livox_ros_driver2
  visualization_msgs
  cv_bridge
)

# 链接 PCL, Python 和 OpenCV
target_link_libraries(fastlio_mapping 
  ${PCL_LIBRARIES} 
  ${Python3_LIBRARIES}
  # [新增] 链接 OpenCV 库
  ${OpenCV_LIBS}
)

rosidl_target_interfaces(fastlio_mapping
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

# -------------------------------------------------------
# 6. 安装规则
# -------------------------------------------------------
install(TARGETS fastlio_mapping
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY PCD
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY scripts
  DESTINATION share/${PROJECT_NAME}
  # 可选：如果你的脚本没有可执行权限，加上这一行确保它被安装为可执行程序
  USE_SOURCE_PERMISSIONS 
)
ament_package()