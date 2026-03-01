cmake_minimum_required(VERSION 3.10)

project(msg_process)

set(CMAKE_CXX_STANDARD  14)

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

#为接口文件生成源码
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AutoaimToDecision.msg"
  "msg/DecisionToAutoaim.msg"
  "msg/DecisonSendData.msg"
  "msg/PursuitData.msg"
  "msg/Pursuit.msg"
  "msg/ReceiveData.msg"
  "msg/DecisionMsg.msg"
   DEPENDENCIES
        std_msgs
        geometry_msgs
        nav_msgs
)

ament_export_dependencies(
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
  rosidl_default_runtime
)

ament_package()