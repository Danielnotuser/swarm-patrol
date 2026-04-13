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
     python3.12-venv"
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
   ros2 launch diff_drive_robot robot.launch.py max_nb_robots:=3"
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
   export ROS_DOMAIN_ID=0 &&\
   ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap /cmd_vel:=/r1/cmd_vel"
```

```bash
    docker exec -it swarmslam bash -c "rm -rf /Swarm-SLAM/src/darp"
```

```bash
    docker exec -it swarmslam bash -c "rm -rf /Swarm-SLAM/install/darp"
```


Копируем darp
```bash
    docker cp ~/Swarm-SLAM/src/darp swarmslam:/Swarm-SLAM/src/darp
```

Устанавливаем зависимости
```bash
    docker exec -it swarmslam bash -c "\
    cd /Swarm-SLAM/src/darp/src && ./Dependencies.sh .venv"
```

Собираем darp package
```bash
    docker exec -it swarmslam bash -c "\
    source /opt/ros/jazzy/setup.bash; 
    source /Swarm-SLAM/install/setup.bash;
    cd Swarm-SLAM && colcon build --packages-select darp"
```

Запускаем darp_node
```bash
   docker exec -it swarmslam bash -c "\
   source /opt/ros/jazzy/setup.bash; \
   source /Swarm-SLAM/install/setup.bash; \
   source /Swarm-SLAM/src/darp/src/.venv/bin/activate; \
   export ROS_DOMAIN_ID=100 &&\
   ros2 run darp darp_bridge_node.py"
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

Запуск визуализации (домен 100):
```bash
    docker exec -it swarmslam bash -c "source /opt/ros/jazzy/setup.bash; \
    source /Swarm-SLAM/install/setup.bash; \
    cd Swarm-SLAM &&\
    export ROS_DOMAIN_ID=100 &&\
    ros2 launch cslam_visualization visualization_lidar.launch.py"
```

Обновляем конфиги для cslam_experiments
```bash
    docker cp ~/Swarm-SLAM/src/cslam_experiments/config/zenoh/zenoh_cslam.json5 swarmslam:/Swarm-SLAM/src/cslam_experiments/config/zenoh/zenoh_cslam.json5 &&\
    docker exec -it swarmslam bash -c "cp /Swarm-SLAM/src/cslam_experiments/config/zenoh/zenoh_cslam.json5 /Swarm-SLAM/install/cslam_experiments/share/cslam_experiments/config/zenoh/zenoh_cslam.json5"
```

Запускаем cslam_experiments для 0 робота с zenoh (домен 0)
```bash
    cd ~/Swarm-SLAM/src/cslam_experiments/docker/ && make swarmslam-lidar ROBOT_ID=0
```

Запускаем cslam_experiments для 1 робота с zenoh (домен 1)
```bash
    cd ~/Swarm-SLAM/src/cslam_experiments/docker/ && make swarmslam-lidar ROBOT_ID=1
```