cmake_minimum_required(VERSION 3.8)
project(serial_process)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(msg_process REQUIRED)

add_executable(receive_process
  src/receive_process.cpp
  serial_data/src/serial_data.cpp
  serial_data/serial_port/src/serial_port.cpp
)

target_include_directories(receive_process PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/serial_data/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/serial_data/serial_port/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(receive_process PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(
  receive_process
  "rclcpp"
  "std_msgs"
  "geometry_msgs"
  "msg_process"
)

install(TARGETS receive_process
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
