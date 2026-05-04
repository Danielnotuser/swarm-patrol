cmake_minimum_required(VERSION 3.8)
project(anomaly_detection)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AnomalyDetected.msg"
  DEPENDENCIES builtin_interfaces geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

install(PROGRAMS
  anomaly_detection_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
