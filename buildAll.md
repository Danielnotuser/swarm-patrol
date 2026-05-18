# Команды для запуска всего, что нужно на данный момент

Запуск докера
```bash
  cd ~/Swarm-SLAM/src/cslam_experiments/docker/ && make cpu_run
```

Скачивание нужных библиотек
```bash
     docker exec -it swarmslam bash -c "
     sudo apt-get update                     &&\
     sudo apt install software-properties-common -y &&\
     sudo add-apt-repository universe -y     &&\
     sudo apt-get update                     &&\
     sudo apt-get install -y                 \
     ros-jazzy-ros-gz                        \
     ros-jazzy-ros-gz-bridge                 \
     ros-jazzy-joint-state-publisher         \
     ros-jazzy-xacro                         \
     ros-jazzy-teleop-twist-keyboard         \
     ros-jazzy-teleop-twist-joy              \
     ros-jazzy-rqt-graph                     \
     ros-jazzy-rtabmap                       \
     ros-jazzy-rtabmap-ros                   \
     python3.12-venv                         \
     ros-jazzy-nav2-bringup"
```

```bash
    docker exec -it swarmslam bash -c "rm -rf /Swarm-SLAM/install/diff_drive_robot"
```

Копирования тестового gazebo
```bash
    docker cp ~/Swarm-SLAM/src/diff_drive_robot/ swarmslam:/Swarm-SLAM/src/
```

Сборка gazebo
```bash
    docker exec -it swarmslam bash -c "\
   source /opt/ros/jazzy/setup.bash; \
   cd /Swarm-SLAM && colcon build --packages-select diff_drive_robot --symlink-install"
```

Запуск gazebo для 0-го робота
```bash
   xhost +local:docker &&\
   docker exec -it swarmslam bash -c "\
   source /opt/ros/jazzy/setup.bash; \
   source /Swarm-SLAM/install/setup.bash;\
   cd /Swarm-SLAM &&\
   export ROS_DOMAIN_ID=0 &&\
   ros2 launch diff_drive_robot robot.launch.py max_nb_robots:=2 world:=two_rooms.world"
```

Запуск gazebo для 1-го робота
```bash
   docker exec -it swarmslam bash -c "\
   source /opt/ros/jazzy/setup.bash; \
   source /Swarm-SLAM/install/setup.bash;\
   cd /Swarm-SLAM &&\
   export ROS_DOMAIN_ID=1 &&\
   ros2 launch diff_drive_robot robot.launch.py max_nb_robots:=2 world:=two_rooms.world"
```

Управляем 0-ым роботом:
```bash
   docker exec -it swarmslam bash -c "\
   source /opt/ros/jazzy/setup.bash; \
   source /Swarm-SLAM/install/setup.bash;\
   export ROS_DOMAIN_ID=0 &&\
   ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap /cmd_vel:=/r0/cmd_vel"
```

Управляем 1-ым роботом:
```bash
   docker exec -it swarmslam bash -c "\
   source /opt/ros/jazzy/setup.bash; \
   source /Swarm-SLAM/install/setup.bash;\
   export ROS_DOMAIN_ID=1 &&\
   ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap /cmd_vel:=/r1/cmd_vel"
```

```bash
    docker exec -it swarmslam bash -c "rm -rf /Swarm-SLAM/src/darp_areas"
```

```bash
    docker exec -it swarmslam bash -c "rm -rf /Swarm-SLAM/install/darp_areas"
```

```bash
    docker cp ~/Swarm-SLAM/src/darp_areas/src/multiRobotPathPlanner.py swarmslam:/Swarm-SLAM/src/darp_areas/src
```

```bash
    docker cp ~/Swarm-SLAM/src/darp_areas/msg/ swarmslam:/Swarm-SLAM/src/darp_areas/msg
```

```bash
    docker cp ~/Swarm-SLAM/src/darp_areas/darp_bridge_node.py swarmslam:/Swarm-SLAM/src/darp_areas/
```

Копируем darp
```bash
    docker cp ~/Swarm-SLAM/src/darp_areas swarmslam:/Swarm-SLAM/src/darp_areas
```

Устанавливаем зависимости
```bash
    docker exec -it swarmslam bash -c "\
    cd /Swarm-SLAM/src/darp_areas/src && ./Dependencies.sh .venv"
```

Собираем darp package
```bash
    docker exec -it swarmslam bash -c "\
    source /opt/ros/jazzy/setup.bash; 
    cd Swarm-SLAM && colcon build --packages-select darp_areas"
```

Запускаем darp_node
```bash
   docker exec -it swarmslam bash -c "\
   source /opt/ros/jazzy/setup.bash &&\
   source /Swarm-SLAM/install/setup.bash &&\
   source /Swarm-SLAM/src/darp_areas/src/.venv/bin/activate &&\
   export ROS_DOMAIN_ID=100 &&\
   ros2 run darp_areas darp_bridge_node.py"
```

```bash
    docker exec -it swarmslam bash -c "rm -rf /Swarm-SLAM/install/cslam_visualization"
```

Копируем cslam_visualization в докер
```bash
    docker cp ~/Swarm-SLAM/src/cslam_visualization/ swarmslam:/Swarm-SLAM/src/cslam_visualization/
```

Собираем cslam_visualization
```bash
    docker exec -it swarmslam bash -c "\
    source /opt/ros/jazzy/setup.bash; \
    source /Swarm-SLAM/install/setup.bash; \
    cd Swarm-SLAM && colcon build --packages-select cslam_visualization"
```

Установка Zenoh-bridge: 
```bash
    docker exec -t swarmslam bash -c "\
    curl -L https://download.eclipse.org/zenoh/debian-repo/zenoh-public-key | sudo gpg --dearmor --yes --output /etc/apt/keyrings/zenoh-public-key.gpg; \
    echo 'deb [signed-by=/etc/apt/keyrings/zenoh-public-key.gpg] https://download.eclipse.org/zenoh/debian-repo/ /' | sudo tee -a /etc/apt/sources.list > /dev/null; \
    sudo apt-get update; sudo apt-get install -y zenohd zenoh-bridge-ros2dds"
```
```bash
    docker exec -it swarmslam bash -c "\
    sudo ln -sf /bin/true /usr/bin/systemctl &&\
    sudo dpkg --configure -a"
```

Запускаем zenoh роутер (запуск перед всем)
```bash
    docker exec -it swarmslam bash -c "zenohd -l tcp/0.0.0.0:7447"
```

Запуск zenoh-bridge визуализации (домен 100):
```bash
    docker exec -it swarmslam bash -c "source /opt/ros/jazzy/setup.bash; \
    source /Swarm-SLAM/install/setup.bash; \
    cd Swarm-SLAM &&\
    export ROS_DOMAIN_ID=100 &&\
    zenoh-bridge-ros2dds -e tcp/127.0.0.1:7447 -c src/cslam_visualization/config/zenoh_cslam.json5"
```

Запуск визуализацию (домен 100):
```bash
    docker exec -it swarmslam bash -c "source /opt/ros/jazzy/setup.bash; \
    source /Swarm-SLAM/install/setup.bash; \
    cd Swarm-SLAM &&\
    export ROS_DOMAIN_ID=100 &&\
    ros2 launch cslam_visualization visualization_lidar.launch.py"
```

Запускаем static publisher
```bash
    docker exec -it swarmslam bash -c "\
   source /opt/ros/jazzy/setup.bash; \
   export ROS_DOMAIN_ID=100 &&\
   ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 robot1_map robot0_map"
```

Обновляем конфиги для cslam_experiments
```bash
    docker cp ~/Swarm-SLAM/src/cslam_experiments/config/zenoh/zenoh_cslam.json5 swarmslam:/Swarm-SLAM/src/cslam_experiments/config/zenoh/zenoh_cslam.json5 &&\
    docker exec -it swarmslam bash -c "cp /Swarm-SLAM/src/cslam_experiments/config/zenoh/zenoh_cslam.json5 /Swarm-SLAM/install/cslam_experiments/share/cslam_experiments/config/zenoh/zenoh_cslam.json5"
```

Запускаем cslam_experiments для 0 робота с zenoh (домен 0)
```bash
    docker exec -it swarmslam bash -c "\
   source /opt/ros/jazzy/setup.bash; \
   source /Swarm-SLAM/install/setup.bash; \
   export ROS_DOMAIN_ID=0 &&\
   ros2 launch cslam_experiments experiment_lidar.launch.py robot_id:=0 max_nb_robots:=2"
```

Запускаем cslam_experiments для 1 робота с zenoh (домен 1)
```bash
    docker exec -it swarmslam bash -c "\
   source /opt/ros/jazzy/setup.bash; \
   source /Swarm-SLAM/install/setup.bash; \
   export ROS_DOMAIN_ID=1 &&\
   ros2 launch cslam_experiments experiment_lidar.launch.py robot_id:=1 max_nb_robots:=2"
```

Запускаем DARP через сервис
```bash
    docker exec -it swarmslam bash -c "\
   source /opt/ros/jazzy/setup.bash; \
   source /Swarm-SLAM/install/setup.bash; \
   export ROS_DOMAIN_ID=100 &&\
   ros2 service call /darp/wake_up darp_areas/srv/WakeUp \"{resolution: 0.5, padding: 0, obstacle_dilation: 1, use_equal_portions: true, portions: [], active_robot_ids: []}\""
```

Запускаем DARP через топик
```bash
    docker exec -it swarmslam bash -c "\
   source /opt/ros/jazzy/setup.bash; \
   source /Swarm-SLAM/install/setup.bash; \
   export ROS_DOMAIN_ID=100 &&\
   ros2 topic pub --once /darp/wake_up darp_areas/msg/WakeUp \"{resolution: 0.5, padding: 0.0, obstacle_dilation: 1, use_equal_portions: true, portions: [], active_robot_ids: [0, 1]}\""
```

```bash
    docker exec -it swarmslam bash -c "rm -rf /Swarm-SLAM/src/nav_darp /Swarm-SLAM/install/nav_darp"
```

Копируем nav_darp
```bash
    docker cp ~/Swarm-SLAM/src/nav_darp swarmslam:/Swarm-SLAM/src/nav_darp
```

Собираем nav_darp package
```bash
    docker exec -it swarmslam bash -c "\
    source /opt/ros/jazzy/setup.bash; 
    cd Swarm-SLAM && colcon build --packages-select nav_darp"
```

Запускаем nav_darp для 0 робота
```bash
   docker exec -it swarmslam bash -c "\
   source /opt/ros/jazzy/setup.bash &&\
   source /Swarm-SLAM/install/setup.bash &&\
   export ROS_DOMAIN_ID=0 &&\
   chmod +x /Swarm-SLAM/src/nav_darp/darp_path_follower.py &&\
   ros2 launch nav_darp robot_nav.launch.py"
```

Запускаем nav_darp для 1 робота
```bash
   docker exec -it swarmslam bash -c "\
   source /opt/ros/jazzy/setup.bash &&\
   source /Swarm-SLAM/install/setup.bash &&\
   export ROS_DOMAIN_ID=1 &&\
   chmod +x /Swarm-SLAM/src/nav_darp/darp_path_follower.py &&\
   ros2 launch nav_darp robot_nav.launch.py robot_id:=1 params_file:=nav2_params_r1.yaml"
```

```bash
    docker exec -it swarmslam bash -c "rm -rf /Swarm-SLAM/install/anomaly_detection /Swarm-SLAM/src/anomaly_detection"
```

Копируем anomaly_detection
```bash
    docker cp ~/Swarm-SLAM/src/anomaly_detection/ swarmslam:/Swarm-SLAM/src/anomaly_detection
```

Собираем anomaly_detection package
```bash
    docker exec -it swarmslam bash -c "\
    source /opt/ros/jazzy/setup.bash; 
    cd Swarm-SLAM && colcon build --packages-select anomaly_detection"
```

Запускаем anomaly_detection для 0 робота
```bash
   docker exec -it swarmslam bash -c "\
   source /opt/ros/jazzy/setup.bash &&\
   source /Swarm-SLAM/install/setup.bash &&\
   export ROS_DOMAIN_ID=0 &&\
   chmod +x /Swarm-SLAM/src/anomaly_detection/anomaly_detection_node.py &&\
   ros2 run anomaly_detection anomaly_detection_node.py"
```

```bash
    docker exec -it swarmslam bash -c "rm -rf /Swarm-SLAM/install/heartbeat_checker /Swarm-SLAM/src/heartbeat_checker"
```

Копируем heartbeat_checker
```bash
    docker cp ~/Swarm-SLAM/src/heartbeat_checker/ swarmslam:/Swarm-SLAM/src/heartbeat_checker
```

Собираем heartbeat_checker package
```bash
    docker exec -it swarmslam bash -c "\
    source /opt/ros/jazzy/setup.bash; 
    cd Swarm-SLAM && colcon build --packages-select heartbeat_checker"
```

Запускаем heartbeat_checker для 0 робота
```bash
   docker exec -it swarmslam bash -c "\
   source /opt/ros/jazzy/setup.bash &&\
   source /Swarm-SLAM/install/setup.bash &&\
   export ROS_DOMAIN_ID=0 &&\
   chmod +x /Swarm-SLAM/src/heartbeat_checker/heartbeat_checker_node.py &&\
   ros2 run heartbeat_checker heartbeat_checker_node.py"
```

Запускаем heartbeat_checker для 1 робота
```bash
   docker exec -it swarmslam bash -c "\
   source /opt/ros/jazzy/setup.bash &&\
   source /Swarm-SLAM/install/setup.bash &&\
   export ROS_DOMAIN_ID=1 &&\
   chmod +x /Swarm-SLAM/src/heartbeat_checker/heartbeat_checker_node.py &&\
   ros2 run heartbeat_checker heartbeat_checker_node.py --ros-args -p robot_id:=1"
```