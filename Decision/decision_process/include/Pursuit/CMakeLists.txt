cmake_minimum_required(VERSION 3.8)
project(pursuit)  # 改为小写以匹配 package.xml

# 查找ROS 2依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenMP REQUIRED)

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 检查OpenMP
if(OpenMP_CXX_FOUND)
    message(STATUS "OpenMP CXX flags: ${OpenMP_CXX_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
else()
    message(FATAL_ERROR "OpenMP not found!")
endif()

# 创建静态库
add_library(pursuit STATIC src/pursuit.cpp)

# 包含目录
target_include_directories(pursuit PUBLIC 
  include
  ${OpenCV_INCLUDE_DIRS}
)

target_include_directories(pursuit PRIVATE 
  ../../include/path
  ../tool/time
)

target_link_libraries(pursuit PRIVATE 
  ${OpenCV_LIBS}
  ${rclcpp_LIBRARIES}
)

# 手动添加 OpenMP
if(OpenMP_CXX_FOUND)
  target_link_libraries(pursuit PRIVATE OpenMP::OpenMP_CXX)
endif()

# 创建测试可执行文件
add_executable(test_pursuit test/test.cpp)

# 手动链接测试程序
target_link_libraries(test_pursuit PRIVATE
  pursuit
  ${OpenCV_LIBS}
  ${rclcpp_LIBRARIES}
)

if(OpenMP_CXX_FOUND)
  target_link_libraries(test_pursuit PRIVATE OpenMP::OpenMP_CXX)
endif()

# 安装目标
install(TARGETS
  pursuit
  test_pursuit
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# 安装头文件
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# 导出依赖
ament_export_dependencies(rclcpp OpenCV)
ament_export_include_directories(include)
ament_export_libraries(pursuit)

ament_package()