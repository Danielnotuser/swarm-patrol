# 22.02.26
Запущен докер, rviz, выведены топики tf, odom, pose, но все в одной точке. 

Нужен датасет лидар данных (KITTI, Graco), однако Graco не дает скачать с их OneDrive, неизвестно почему.

На данный момент выбран датасет с KITTI:  projected raw LiDaR scans data set (5 GB)
(https://www.cvlibs.net/datasets/kitti/eval_depth.php?benchmark=depth_completion)

# 01.03.26

Запущен Gazebo в докере (make gazebo)

Все еще неясно как скормить лидар данные и правильные ли скачаны

# 02.03.26

Запущен докер, склонирован cslam_visualization в src.
```bash
  docker exec -it swarmslam bash -c "cd Swarm-SLAM/src && git clone https://github.com/lajoiepy/cslam_visualization.git"
```
Произведена следующая последовательность команд:

Установка rtabmap
```bash
    docker exec -it swarmslam bash -c "source /opt/ros/jazzy/setup.bash; source /Swarm-SLAM/install/setup.bash; cd Swarm-SLAM/src/cslam_visualization && sudo apt-get update && sudo apt-get install -y ros-jazzy-rtabmap ros-jazzy-rtabmap-ros"
```
Build cslam_visualization
```bash
    docker exec -it swarmslam bash -c "source /opt/ros/jazzy/setup.bash; source /Swarm-SLAM/install/setup.bash; cd Swarm-SLAM/src/cslam_visualization && colcon build"
```
Установка Zenoh-bridge: (не получилось)
```bash
    docker exec -t swarmslam bash -c "curl -L https://download.eclipse.org/zenoh/debian-repo/zenoh-public-key | sudo gpg --dearmor --yes --output /etc/apt/keyrings/zenoh-public-key.gpg; \
    echo 'deb [signed-by=/etc/apt/keyrings/zenoh-public-key.gpg] https://download.eclipse.org/zenoh/debian-repo/ /' | sudo tee -a /etc/apt/sources.list > /dev/null; \
    sudo apt-get update; sudo apt-get install -y zenohd zenoh-plugin-ros2dds"
```
Копирование папки install в docker:
```bash
  docker cp ~/Swarm-SLAM/install/cslam_visualization swarmslam:/Swarm-SLAM/install
```
Преднастройка визуализации (rviz, gazebo, xlocal):  **(upd. 06.03.26)**
```bash
    cd src/cslam_experiments/docker
    make viz
```
Запуск визуализации:
```bash
    docker exec -it swarmslam bash -c "source /opt/ros/jazzy/setup.bash; source /Swarm-SLAM/install/setup.bash; cd Swarm-SLAM/src/cslam_visualization && ros2 launch cslam_visualization visualization_lidar.launch.py"
```

### Итог: запустился rviz, но пустой остается даже после запуска make swarmslam-lidar

# 12.03.26

Пробуем визуализацию с помощью rerun

Запущен докер (make cpu_run), клонируем cslam_visualization в src.
```bash
    docker exec -it swarmslam bash -c "\
    cd Swarm-SLAM/src &&\
    git clone https://github.com/lajoiepy/cslam_visualization.git"
```

Установка rtabmap
```bash
    docker exec -it swarmslam bash -c "\
    cd Swarm-SLAM/src/cslam_visualization &&\
    sudo apt-get update &&\
    sudo apt-get install -y ros-jazzy-rtabmap ros-jazzy-rtabmap-ros"
```

Установка rerun
```bash
    docker exec -it swarmslam bash -c "\
    sudo apt install python3-venv -y &&\
    python3 -m venv .venv &&\
    source .venv/bin/activate &&\
    pip install rerun-sdk"
```
Преднастройка визуализации (rviz, gazebo, xlocal):  
```bash
    cd src/cslam_experiments/docker
    make viz
```

Запуск rerun
```bash
  docker exec -it swarmslam bash -c "source .venv/bin/activate && rerun" 
```
### **失敗しました** rerun not supported для докеров :)

Подготовка визуализации **(unused/unchecked)**
```bash
  docker exec -it swarmslam bash -c "\
    cd Swarm-SLAM/src/cslam_visualization &&\
    git checkout rerun_viz &&\
    vcs import src < swarmslam_visualization.repos &&\
    cd docker/devel/ &&\
    make build &&\
    make run &&\
    make attach &&\
    colcon build"
```

Запуск launch визуализации **(unused/unchecked)**

```bash
    docker exec -it swarmslam bash -c "\
    source /opt/ros/jazzy/setup.bash; \
    source /Swarm-SLAM/install/setup.bash;\
    cd Swarm-SLAM/src/cslam_visualization &&\
    ros2 launch rerun_lidar.launch.py" 
```

# 14.03.26

All in all, I've decided to download again the KITTI datasets (GrAco still does not work) for the cslam_experiments (exactly: launch/datasets_experiments/kitti_lidar.launch.py)

The whole problem with these datasets is they do not want to download quickly (though now it is okay) and I lack information about what are the actual datasets needed for the launch file.

The right ones are probably the odometry KITTI ones, but the size of lidar scans dataset is wildly big (80GB). Of course, I have no idea how I can possibly copy them to the docker container...

Looks like the **ros2bag** is needed.

To be continued...

# 15.03.26

Я отыскал рабочую ссылку на скачивание ros2bag данных у Graco (на их гитхабе есть спасательная ссылка)

Используется GrAco/ground/ros2/ground-01_0.db3

C onedrive очень долго скачивалось

# 16.03.26

Повторная попытка colcon build на локальной машине, ошибка с gtsam продолжается.

Пока в докере пробуем:
проводим все то же самое с cslam_visualization, как в 02.03.26

После этого запускаем swarm_slam c graco датасетом: (не вышло, то ли нерпавильно скачался ros2bag, то ли ??? (проверялся с помощью ros2 bag info, ошибка database disk image is malformed))

```bash 
    docker cp ~/Swarm-SLAM/src/cslam_experiments/data swarmslam:/Swarm-SLAM/src/cslam_experiments/data
    cd src/cslam_experiments/docker && make graco
```

Gazebo:

```bash
    docker cp ~/Swarm-SLAM/src/cslam_experiments/launch/gazebo/ swarmslam:/Swarm-SLAM/src/cslam_experiments/launch/gazebo/
```

```bash
     docker exec -it swarmslam bash -c "\
    source /opt/ros/jazzy/setup.bash; \
    source /Swarm-SLAM/install/setup.bash;\
    cd /Swarm-SLAM && colcon build --packages-select cslam_experiments"
```

```bash
     docker exec -it swarmslam bash -c "\
    source /opt/ros/jazzy/setup.bash; \
    source /Swarm-SLAM/install/setup.bash;\
    ros2 launch cslam_experiments gazebo.launch.py"
```

Почему-то пишет "file 'gazebo.launch.py' was found more than once in the share directory of package 'cslam_experiments': ['/Swarm-SLAM/install/cslam_experiments/share/cslam_experiments/launch/gazebo/gazebo.launch.py', '/Swarm-SLAM/install/cslam_experiments/share/cslam_experiments/launch/gazebo/gazebo/gazebo.launch.py']"

```bash
    docker exec -it swarmslam bash -c "rm -rf /Swarm-SLAM/install/cslam_experiments/share/cslam_experiments/launch/gazebo/gazebo/"
```

Полная очистка и перезапуск colcon build:
```bash
    docker exec -it swarmslam bash -c "\
    cd /Swarm-SLAM &&\
    rm -rf build log install &&\
    colcon build"
```


Найден еще один датасет (https://docs.ros.org/en/noetic/api/ov_core/html/gs-datasets.html)

KAIST Urban Dataset: скинули на почту sample data, но она в bin файлах и, чтобы ее преобразовать в ros2bag, нужно преобразовать сначала в ros1bag, чего я не могу сделать на своей версии убунту (22.04)

Следующий: (https://figshare.com/articles/dataset/LiRAR_data_for_RealRecon_/28473722) хранит ros1bag'и, что также тяжело, единственный вариант - это уже в докере преобразовывать

```bash
    docker cp ~/Swarm-SLAM/src/cslam_experiments/data/RealRecon_Port/ swarmslam:/Swarm-SLAM/src/cslam_experiments/data/RealRecon_Port/
```

Преобразование .bag ROS1 в .db3 ROS2:

```bash
   docker exec -it swarmslam bash -c "\
   sudo apt-get update && sudo apt-get install python3.12-venv -y &&\
   cd /Swarm-SLAM/src/cslam_experiments/ &&\
   python3 -m venv .venv &&\
   source .venv/bin/activate &&\
   pip install rosbags>=0.9.11 &&\
   rosbags-convert --src data/RealRecon_Port/lidar_data_Port.bag --dst data/RealRecon_Port/lidar_data_Port.db3"
```

```bash
   docker exec -it swarmslam bash -c "\
   source /opt/ros/jazzy/setup.bash; \
   source /Swarm-SLAM/install/setup.bash;\
   cd /Swarm-SLAM/src/cslam_experiments/data/RealRecon_Port/ &&\
   ros2 bag info lidar_data_Port.db3 -s sqlite3"
```

```bash
   docker exec -it swarmslam bash -c "\
   source /opt/ros/jazzy/setup.bash; \
   source /Swarm-SLAM/install/setup.bash;\
   cd /Swarm-SLAM/src/cslam_experiments/data/RealRecon_Port/ &&\
   ls -la"
```

Получилось!

```
Files:             lidar_data_Port.db3.db3
Bag size:          531.7 MiB
Storage id:        sqlite3
ROS Distro:        rosbags
Duration:          140.465517177s
Start:             Jan 18 2025 15:26:20.440191436 (1737213980.440191436)
End:               Jan 18 2025 15:28:40.905708613 (1737214120.905708613)
Messages:          2812
Topic information: Topic: /lidar/odom | Type: nav_msgs/msg/Odometry | Count: 1406 | Serialization Format: cdr
                   Topic: /lidar/point_cloud | Type: sensor_msgs/msg/PointCloud2 | Count: 1406 | Serialization Format: cdr
Service:           0
Service information: 

```

Теперь стоит задача создания real_recon_lidar.launch.py файла (наподобие graco\kitti в datasets_experiments)

# 17.03.26

Продолжаем начатое 16.03

немного подправил название в командах там.

копируем launch файлы для нашего bag:
```bash
    docker cp ~/Swarm-SLAM/src/cslam_experiments/launch/datasets_experiments/real_recon_lidar.launch.py swarmslam:/Swarm-SLAM/src/cslam_experiments/launch/datasets_experiments/real_recon_lidar.launch.py
    docker cp ~/Swarm-SLAM/src/cslam_experiments/config/real_recon_lidar.yaml swarmslam:/Swarm-SLAM/src/cslam_experiments/config/eal_recon_lidar.yaml
    docker cp ~/Swarm-SLAM/src/cslam_experiments/launch/odometry/rtabmap_real_recon_lidar_odometry.launch.py swarmslam:/Swarm-SLAM/src/cslam_experiments/launch/odometry/rtabmap_real_recon_lidar_odometry.launch.py
    docker cp ~/Swarm-SLAM/src/cslam_experiments/launch/sensors/bag_real_recon.launch.py swarmslam:/Swarm-SLAM/src/cslam_experiments/launch/sensors/bag_real_recon.launch.py
```

запускаем launch файл
```bash
    docker exec -it swarmslam bash -c "\
   source /opt/ros/jazzy/setup.bash; \
   source /Swarm-SLAM/install/setup.bash;\
   cd /Swarm-SLAM && colcon build --packages-select cslam_experiments &&\
   ros2 launch cslam_experiments real_recon_lidar.launch.py robot_id:=0 max_nb_robots:=1"
```

Все запускается, но вывод говорит, что ничего не публикуется в топики, какая-то ошибка в launch файлах