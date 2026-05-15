cmake_minimum_required(VERSION 3.8)
project(nav_darp)

# Подключаем стандартные модули
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Устанавливаем Python-скрипт (ноду) в lib/nav_darp
install(PROGRAMS
  darp_path_follower.py realtime_pointcloud.py odom_to_map.py static_tf_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

# Устанавливаем launch-файлы
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Устанавливаем конфигурационные файлы
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  behavior_trees
  DESTINATION share/${PROJECT_NAME}
)

# Проверяем, что всё необходимое найдено (опционально)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Упаковываем
ament_package()