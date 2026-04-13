cmake_minimum_required(VERSION 3.8)
project(darp)

# --- Dependencies ---
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# --- Generate service ---
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/WakeUp.srv"
)

ament_export_dependencies(rosidl_default_runtime)

# --- Install Python node ---
install(PROGRAMS
  darp_bridge_node.py
  DESTINATION lib/${PROJECT_NAME}/
)

install(DIRECTORY
  src
  DESTINATION share/${PROJECT_NAME}/
)

# --- Package ---
ament_package()


