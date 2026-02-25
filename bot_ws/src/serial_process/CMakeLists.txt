cmake_minimum_required(VERSION 3.10)
project(serial_process)

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_subdirectory(serial_data)

include_directories(include)

add_executable(send_process src/send_process.cpp)

ament_target_dependencies(send_process
    rclcpp
    std_msgs
    geometry_msgs
)

target_link_libraries(send_process
    serial_data
)

install(TARGETS
  send_process
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()