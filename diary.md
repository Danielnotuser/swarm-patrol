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

Установка rtabmap (не используется, т.к. устнавливается при вызове газебо)
```bash
    docker exec -it swarmslam bash -c "source /opt/ros/jazzy/setup.bash; source /Swarm-SLAM/install/setup.bash; cd Swarm-SLAM/src/cslam_visualization && sudo apt-get update && sudo apt-get install -y ros-jazzy-rtabmap ros-jazzy-rtabmap-ros"
```

Build cslam_visualization вместе со всеми
```bash
    docker exec -it swarmslam bash -c "source /opt/ros/jazzy/setup.bash; source /Swarm-SLAM/install/setup.bash; cd Swarm-SLAM && colcon build --packages-select cslam_visualization"
```
Установка Zenoh-bridge: (02.03.26)
```bash
    docker exec -t swarmslam bash -c "curl -L https://download.eclipse.org/zenoh/debian-repo/zenoh-public-key | sudo gpg --dearmor --yes --output /etc/apt/keyrings/zenoh-public-key.gpg; \
    echo 'deb [signed-by=/etc/apt/keyrings/zenoh-public-key.gpg] https://download.eclipse.org/zenoh/debian-repo/ /' | sudo tee -a /etc/apt/sources.list > /dev/null; \
    sudo apt-get update; sudo apt-get install -y zenohd zenoh-bridge-ros2dds"
```

```bash
    docker exec -it swarmslam bash -c "\
    sudo ln -sf /bin/true /usr/bin/systemctl &&\
    sudo dpkg --configure -a"
```

Преднастройка визуализации (rviz, gazebo, xlocal):  **(upd. 06.03.26)** (не используется, т.к. устнавливается при вызове газебо)
```bash
    cd src/cslam_experiments/docker
    make viz
```
Запуск визуализации:
```bash
    docker exec -it swarmslam bash -c "source /opt/ros/jazzy/setup.bash; \
    source /Swarm-SLAM/install/setup.bash; \
    cd Swarm-SLAM &&\
    export ROS_DOMAIN_ID=100 &&\
    ros2 launch cslam_visualization visualization_lidar.launch.py"
```

Запуск zenoh-bridge:
```bash
    docker exec -it swarmslam bash -c "source /opt/ros/jazzy/setup.bash; \
    source /Swarm-SLAM/install/setup.bash; \
    cd Swarm-SLAM &&\
    export ROS_DOMAIN_ID=100 &&\
    zenoh-bridge-ros2dds -e tcp/127.0.0.1:7447 -c src/cslam_visualization/config/zenoh_cslam.json5"
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

# 24.03.26

Разбирался отдельно с симуляцией gazebo локально, было много конфликтов с gazebo_ros (оказалось это штука от gazebo classic тянулась), менял на ros_gz sim и тд

в след раз запущу мир gazebo в докере с cslam_experiments

# 25.03.26

запускаем докер, добавляем gazebo 

prerequisites:
```bash
     docker exec -it swarmslam bash -c "
     sudo apt-get update                     &&\
     sudo apt install software-properties-common -y &&\
     sudo add-apt-repository universe        &&\
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
     ros-jazzy-rtabmap-ros"
```
сам gazebo repo
```bash
    docker cp ~/Swarm-SLAM/src/diff_drive_robot/ swarmslam:/Swarm-SLAM/src/
```
собираем:
```bash
    docker exec -it swarmslam bash -c "\
   source /opt/ros/jazzy/setup.bash; \
   cd /Swarm-SLAM && colcon build --packages-select diff_drive_robot --symlink-install"
```
запускаем:
```bash
   xhost +local:docker &&\
   docker exec -it swarmslam bash -c "\
   source /opt/ros/jazzy/setup.bash; \
   source /Swarm-SLAM/install/setup.bash;\
   cd /Swarm-SLAM &&\
   export ROS_DOMAIN_ID=1 &&\
   ros2 launch diff_drive_robot robot.launch.py max_nb_robots:=3"
```

управляем 0-ым роботом:
```bash
   docker exec -it swarmslam bash -c "\
   source /opt/ros/jazzy/setup.bash; \
   source /Swarm-SLAM/install/setup.bash;\
   export ROS_DOMAIN_ID=0 &&\
   ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap /cmd_vel:=/r0/cmd_vel"
```
управляем 1-ым роботом:
```bash
   docker exec -it swarmslam bash -c "\
   source /opt/ros/jazzy/setup.bash; \
   source /Swarm-SLAM/install/setup.bash;\
   export ROS_DOMAIN_ID=1 &&\
   ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap /cmd_vel:=/r1/cmd_vel"
```

проверяем:
```bash
    docker exec -it swarmslam bash -c "\
   source /opt/ros/jazzy/setup.bash; \
   gz topic -l"
```
```bash
    docker exec -it swarmslam bash -c "\
   source /opt/ros/jazzy/setup.bash; \
   gz topic -e -t /pointcloud/points"
```
```bash
    docker exec -it swarmslam bash -c "\
   source /opt/ros/jazzy/setup.bash; \
   export ROS_DOMAIN_ID=1 &&\
   ros2 topic list -t"
```
```bash
    docker exec -it swarmslam bash -c "\
   source /opt/ros/jazzy/setup.bash; \
   source /Swarm-SLAM/install/setup.bash; \
   export ROS_DOMAIN_ID=0 &&\
   rqt_graph"
```
```bash
    docker exec -it swarmslam bash -c "\
   source /opt/ros/jazzy/setup.bash; \
   source /Swarm-SLAM/install/setup.bash; \
   export ROS_DOMAIN_ID=0 &&\
   sudo apt-get update && sudo apt-get install ros-jazzy-rqt-tf-tree -y
   ros2 run rqt_tf_tree rqt_tf_tree"
```
```bash
    docker exec -it swarmslam bash -c "\
   source /opt/ros/jazzy/setup.bash; \
   source /Swarm-SLAM/install/setup.bash; \
   export ROS_DOMAIN_ID=100 &&\
   ros2 topic echo /cslam/viz/cloudmarker --once"
```
```bash
    docker exec -it swarmslam bash -c "\
   source /opt/ros/jazzy/setup.bash; \
   ros2 node list --all"
```
```bash
    docker exec -it swarmslam bash -c "\
   source /opt/ros/jazzy/setup.bash; \
   ros2 node info /r0/cslam_map_manager"
```

ROS_DOMAIN_ID=


в топик постится, но какой результат должен быть? почему у namespace r0 topic r0/pointcloud есть, но r0/scan нету? Только 3Д?

# 27.03.2026

в ходе вчерашнего расследования, выяснилось (пока не точно), что для работы Swarm-SLAM нужен лидар, выдающий PointCloud2 вместо LaserScan

пока поменял diff_drive_robot пакет на использование gpu_lidar (но не уверен, что получится использовать)

в результате pointcloud ничего не выводит (газебо успешно запускается), еще хотел посмотреть ноды активные (ros2 node info), но по какой-то причине list их видит, а info нет

# 28.03.26

запустить swarm slam vizualization, static publisher

+ несколько роботов

# 30.03.26

Успешно запущен cslam_visualization с визуализацией pose graph и pointcloud'a

Изменены launch файлы и urdf для запуска нескольких роботов в газебо, однако почему-то в cslam_visualization граф поз и поинтклауд не виден

Остается задача запуска нескольких роботов и static publisher (нужно понять откуда куда трансформировать, по идее diff_robot_0/base_link/lidar_sensor в robot0_map/)

# 31.03.26

запускаем static publisher

```bash
    docker exec -it swarmslam bash -c "\
   source /opt/ros/jazzy/setup.bash; \
   export ROS_DOMAIN_ID=100 &&\
   ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 robot1_map robot0_map"
```

почему-то в cslam_experiments/launch/robot_experiments/experiment_lidar.launch.py ROS_DOMAIN_ID задается как номер робота, что неправильно (?), ведь нужен один домен для всех роботов,
поменяем:
```bash
    docker cp ~/Swarm-SLAM/src/cslam_experiments/launch/robot_experiments/experiment_lidar.launch.py swarmslam:/Swarm-SLAM/src/cslam_experiments/launch/robot_experiments/experiment_lidar.launch.py
```
```bash
    docker exec -it swarmslam bash -c "\
   source /opt/ros/jazzy/setup.bash; \
   source /Swarm-SLAM/install/setup.bash; \
   cp /Swarm-SLAM/src/cslam_experiments/launch/robot_experiments/experiment_lidar.launch.py /Swarm-SLAM/install/cslam_experiments/share/cslam_experiments/launch/robot_experiments/
   ros2 launch cslam_experiments experiment_lidar.launch.py robot_id:=1 max_nb_robots:=3"
```

даже с одним ROS_DOMAIN_ID только 0-й робот выводится в cslam_visualization, хотя отличия от 1-го робота только в неймспейсе и robot_id

также на rqt_graph не выводится /r1 ноды и топики по какой-то причине (make swarmslam-lidar запущен)

предполагаю, что либо experiment_lidar.launch.py только для одного робота предназаначен (что вряд ли) либо я как-то неправильно запускаю второй, потому что в списке топиков не появляются такие же cslam топики, как у r0

# 01.04.26

с копированием обновленного experment_lidar.launch.py в install/share в визуализации появились точки другого цвета, отвечающие за 1-й робот, однако при попытке вывода графа поз общего выводится только 0-е позы и поинтклауд (если находится в robot0_map фрейме)

при публикации static_transform из robot1_map в robot0_map видны прошлые позы 1-го робота, но новые не появляются

при выключении swarm-slam 0-го робота внезапно на rviz появились все позы 1-го робота с поинтклаудом

# 02.04.26

отношение TF
```bash
    docker exec -it swarmslam bash -c "\
   source /opt/ros/jazzy/setup.bash; \
   source /Swarm-SLAM/install/setup.bash; \
   export ROS_DOMAIN_ID=100 &&\
   ros2 run tf2_tools view_frames"
```
```bash
    docker cp swarmslam:frames_2026-04-27_23.04.22.pdf  ~/Swarm-SLAM/data/
```
```bash
    docker exec -it swarmslam bash -c "\
   source /opt/ros/jazzy/setu0p.bash; \
   source /Swarm-SLAM/install/setup.bash; \
   ls -la"
```

доп логирование сообщения в колбеке визуализации поз графа (копируем полностью, чтобы еще конфиги обновить)
```bash
    docker cp ~/Swarm-SLAM/src/cslam_visualization/ swarmslam:/Swarm-SLAM/src/cslam_visualization/
```

# 03.04.26

похоже, что работа без zenoh bridge не предусмотрена (точнее не работает просто), пытаемся настроить zenoh в докере:

Установка Zenoh-bridge: (02.03.26)
```bash
    docker exec -t swarmslam bash -c "curl -L https://download.eclipse.org/zenoh/debian-repo/zenoh-public-key | sudo gpg --dearmor --yes --output /etc/apt/keyrings/zenoh-public-key.gpg; \
    echo 'deb [signed-by=/etc/apt/keyrings/zenoh-public-key.gpg] https://download.eclipse.org/zenoh/debian-repo/ /' | sudo tee -a /etc/apt/sources.list > /dev/null; \
    sudo apt-get update; sudo apt-get install -y zenohd zenoh-plugin-ros2dds"
```

```bash
    docker exec -it swarmslam bash -c "\
    sudo ln -sf /bin/true /usr/bin/systemctl &&\
    sudo dpkg --configure -a"
```

```bash
    docker exec -it swarmslam bash -c "\
    which zenohd;
    which zenoh-plugin-ros2dds"
```

zenoh получилось запустить, но почему-то визуализация не видит оба робота (в ROS_DOMAIN_ID=100 запущена визуализация с мостом)

# 04.04.26

Добавляем явный роутер (запуск перед всем)
```bash
    docker exec -it swarmslam bash -c "zenohd -l tcp/0.0.0.0:7447"
```
обновляем конфиги 
```bash
    docker cp ~/Swarm-SLAM/src/cslam_experiments/config/zenoh/zenoh_cslam.json5 swarmslam:/Swarm-SLAM/src/cslam_experiments/config/zenoh/zenoh_cslam.json5
```
```bash
    docker exec -it swarmslam bash -c "\
   source /opt/ros/jazzy/setup.bash; \
   source /Swarm-SLAM/install/setup.bash; \
   cp /Swarm-SLAM/src/cslam_experiments/config/zenoh/zenoh_cslam.json5 /Swarm-SLAM/install/cslam_experiments/share/cslam_experiments/config/zenoh/zenoh_cslam.json5 &&\
   export ROS_DOMAIN_ID=1 &&\
   ros2 launch cslam_experiments experiment_lidar.launch.py robot_id:=1 max_nb_robots:=3"
```

почитать исходники, доделать визуализации, DARP начать

# 07.04.26

получилось запустить визуализацию для обоих роботов в одном фрейме для этого:
+ запущена визуализация с мостом в домене 100
+ роботы в доменах 0 и 1
+ газебо в доменах 0 и 1 со своими телеопами (! в этом раньше была ошибка, т.к. газебо был запущен в одном домене (по умолч. =0))
+ и пришлось все равно в 100 домене сделать static_transform robot1_map robot0_map, чтобы видеть обоих одновременно, но по отдельности их можно видеть в разных фреймах (раньше даже это не работало)

скорее всего при использовании одного домена какие-то топики перетираются, возможно tf

постараться пофиксить газебо конфиги, чтобы в одном домене можно было бы работать, хотя наверно не обязательно ¯\\\_(ツ)_/¯

# 11.04.26

```bash
    docker exec -it swarmslam bash -c "\
   source /opt/ros/jazzy/setup.bash; \
   source /Swarm-SLAM/install/setup.bash; \
   export ROS_DOMAIN_ID=0 &&\
   rviz2"
```

доделать darp, используя cslam/viz/pointcloud_marker в качестве входа, также сервис для запуска

# 13.04.26

обновлен darp_node: добавлен сервис WakeUp для запуска DARP в спящей darp_node, 

вход: /cslam/viz/cloudmaker

выход: /rN/darp/route и /rN/darp/area

запуск ноды должен происходить перед всеми нодами, так как он слушает визуализацию. будет запускаться в домене визуализации - 100

```bash
    docker cp ~/Swarm-SLAM/src/darp swarmslam:/Swarm-SLAM/src/darp
```

```bash
    docker exec -it swarmslam bash -c "\
    source /opt/ros/jazzy/setup.bash; \
    source /Swarm-SLAM/install/setup.bash; \
    cd Swarm-SLAM && colcon build --packages-select darp"
```

```bash
   docker exec -it swarmslam bash -c "\
   source /opt/ros/jazzy/setup.bash; \
   source /Swarm-SLAM/install/setup.bash; \
   export ROS_DOMAIN_ID=100 &&\
   ros2 run darp darp_node"
```

# 19.04.26

За несколько дней успешно запустилась darp node, но пока работает не совсем верно (ошибка запуска darp была в конфликте имен ros2 пакета и питон библиотеки darp - ros2 пакет переименован в darp_areas)

Запускаем DARP через сервис
```bash
    docker exec -it swarmslam bash -c "\
   source /opt/ros/jazzy/setup.bash; \
   source /Swarm-SLAM/install/setup.bash; \
   export ROS_DOMAIN_ID=100 &&\
   ros2 service call /darp/wake_up darp_areas/srv/WakeUp \"{resolution: 0.05, padding: 0.2, obstacle_dilation: 0.1, use_equal_portions: true, portions: []}\""
```

DARP работает исправно

nav2 

# 20.04.26

Возможно, DARP обозначает начальные позиции роботов неверно, надо поизучать

Добавлена nav_darp нода, пока в тестинге

# 25.04.26

nav2, darp step, anomaly detection, frontier exploration, heartbeat

в аномалиях добавить вывод маркеров в рвиз (как /cslam/viz/cloudmarker)

frontier - обвод контуров и нахождение позы открытой местности
расстояние между роботов через позы (начинать по одному возможно)

heartbeat - слушаем соседа и если не слышали долго, то считаем мертвыми

# 27.04.26

пофикшены многочисленные ошибки nav2 за счет неиспользуемых нод в nav2_bringup. 

в итоге поменяно на список отдельных нужных нод (пока минимально нужный список),
также добавлены в zenoh конфиги топики /tf для того, чтобы во всех доменах все знали о всех фреймах (тоже для нав2)

сейчас проблема преобразования robot0_map и robot0_keyframe0 в виде:

[controller_server-1] [ERROR] [1777331241.786831585] [tf_help]: Transform data too old when converting from robot0_map to robot0_keyframe0

поменял у gazebo launch файла use_sim_time = false и в публикатора Path в DARP сделал так, чтобы timestamp у поз и пути в целом поменялись на время нынешнее, но, похоже, не то