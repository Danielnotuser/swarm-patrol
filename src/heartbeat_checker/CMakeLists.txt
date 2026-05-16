cmake_minimum_required(VERSION 3.8)
project(heartbeat_checker)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LostRobots.msg"
  DEPENDENCIES std_msgs
)

install(PROGRAMS
  heartbeat_checker_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
