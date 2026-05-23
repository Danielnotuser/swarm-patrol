cmake_minimum_required(VERSION 3.8)
project(frontier_exploration)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(frontier_exploration)

install(PROGRAMS
  frontier_exploration/frontier_finder.py
  frontier_exploration/coordinator.py
  frontier_exploration/exploration_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()