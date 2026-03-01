cmake_minimum_required(VERSION 3.5)
project(decision_process)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenMP REQUIRED)
get_filename_component(_repo_root "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
set(_msg_process_cmake_hint "${_repo_root}/msg_process/install/msg_process/share/msg_process/cmake")
if(NOT msg_process_DIR AND EXISTS "${_msg_process_cmake_hint}/msg_processConfig.cmake")
  set(msg_process_DIR "${_msg_process_cmake_hint}")
endif()
find_package(msg_process REQUIRED)

add_subdirectory(tinyxml2)
add_subdirectory(include/semantic_map)
add_subdirectory(include/Pursuit)

if(OpenMP_CXX_FOUND)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

include_directories(
  include
  log
  config
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(decision_process src/decision_process.cpp)

ament_target_dependencies(decision_process
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  tf2
  behaviortree_cpp
  msg_process
)

target_link_libraries(decision_process
  decision_semantic_map
  pursuit
  yaml-cpp
  ${OpenCV_LIBS}
)

target_include_directories(decision_process PRIVATE include)

install(TARGETS
  decision_process
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

# install config files (so runtime lookup via ament_index_cpp can find them)
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# install XML files and Pursuit maps into the package share so runtime lookups
# (which use ament_index_cpp get_package_share_directory) can find them in
# the installed layout.
install(DIRECTORY XML/
  DESTINATION share/${PROJECT_NAME}/XML
)

install(DIRECTORY include/Pursuit/map/
  DESTINATION share/${PROJECT_NAME}/include/Pursuit/map
)

# install semantic_map xml files into share as well
install(DIRECTORY include/semantic_map/xml/
  DESTINATION share/${PROJECT_NAME}/include/semantic_map/xml
)

ament_package()
