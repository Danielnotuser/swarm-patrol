# Swarm-Patrol: Мульти-роботный патруль, исследование и покрытие территории

<p align="center">
  <em>Мульти-роботная система, объединяющая Collaborative SLAM, исследование на основе фронтиров, DARP-разделение территории, детекцию аномалий и автономную навигацию Nav2.</em>
</p>

---

## Обзор

**Swarm-Patrol** интегрирует несколько open-source фреймворков в единый стек для мульти-роботной автономии:

- **[Swarm-SLAM](https://github.com/MISTLab/Swarm-SLAM)** — Разреженный децентрализованный коллаборативный SLAM. Разработан в **MISTLab (École Polytechnique de Montréal)** под руководством **Pierre-Yves Lajoie** и **Giovanni Beltrame**. Позволяет нескольким роботам совместно строить карту без центрального сервера, используя механизм приоритизации меж-роботных циклов замыкания (vertex-cover / broker).
- **[DARP](https://github.com/athakapo/DARP)** — **Divide Areas Algorithm for Robots** от **Athanasios Kapoutsis et al.** (CC BY-NC 4.0). Делит общую карту занятости на равные или взвешенные участки между роботами с построением маршрутов полного покрытия (Spanning Tree Coverage, STC).
- **Nav2** — Стек навигации ROS 2 для планирования пути, избегания препятствий и автономного перемещения.
- **Zenoh** — Эффективная маршрутизация DDS между доменами роботов.
- **Frontier Exploration** — Автономное исследование карты, когда маршруты DARP недоступны.

### Ключевые возможности

- **Децентрализованный SLAM**: Без центрального сервера; связь peer-to-peer через Zenoh.
- **Мульти-роботная координация**: Протокол резервирования предотвращает выбор одной и той же области разными роботами (во время фронтирной навигации).
- **Адаптивное исследование**: Переключение между исследованием фронтиров и следованием маршрутам DARP.
- **Детекция аномалий**: Сравнение LiDAR-сканов с картой занятости в реальном времени для обнаружения неожиданных препятствий (появление/исчезновение).
- **Мониторинг Heartbeat**: Обнаружение отказа робота и запуск переразделения территории.
- **ROS 2 + Gazebo симуляция**: Полный стек симуляции с настраиваемыми мирами и количеством роботов.

---

## Быстрый старт (симуляция в Gazebo)

### Требования

- ROS 2 Jazzy (или Humble)
- Gazebo (Ignition Fortress или новее)
- [Зависимости Swarm-SLAM](https://github.com/MISTLab/Swarm-SLAM) (GTSAM 4.2, TEASER++, Zenoh)

### Сборка

```bash
colcon build --symlink-install
source install/setup.bash
```

### Запуск симуляции

**Терминал 1 — Gazebo с 3 роботами:**
```bash
ros2 launch diff_drive_robot robot.launch.py robot_count:=3 world:=corridor
```

**Терминал 2 — C-SLAM + Zenoh bridge для каждого робота:**
```bash
ros2 launch cslam_experiments experiment_lidar.launch.py robot_id:=0 max_nb_robots:=3
# В отдельных терминалах:
ros2 launch cslam_experiments experiment_lidar.launch.py robot_id:=1 max_nb_robots:=3
ros2 launch cslam_experiments experiment_lidar.launch.py robot_id:=2 max_nb_robots:=3
```

**Терминал 3 — Nav2 навигация для каждого робота:**
```bash
ros2 launch nav_darp robot_nav.launch.py robot_id:=0
# В отдельных терминалах:
ros2 launch nav_darp robot_nav.launch.py robot_id:=1
ros2 launch nav_darp robot_nav.launch.py robot_id:=2
```

**Терминал 4 — Frontier exploration для каждого робота:**
```bash
ros2 run frontier_exploration exploration_node --ros-args -p robot_id:=0 -p robot_count:=3
# В отдельных терминалах:
ros2 run frontier_exploration exploration_node --ros-args -p robot_id:=1 -p robot_count:=3
ros2 run frontier_exploration exploration_node --ros-args -p robot_id:=2 -p robot_count:=3
```

**Терминал 5 — DARP bridge (один экземпляр в домене каждого робота; активен только у брокера):**
```bash
# Запускается в домене робота-брокера (например, robot 0):
ros2 run darp_areas darp_bridge_node --ros-args -p robot_count:=3
```

**Терминал 6 — Визуализация (на domain 100):**
```bash
ROS_DOMAIN_ID=100 ros2 launch cslam_visualization visualization_lidar.launch.py
```

### Реальные роботы

На реальных роботах DARP запускается **в домене каждого робота** (не в domain 100). Только экземпляр DARP **робота-брокера** активен в каждый момент времени — он вычисляет разделение территории и публикует маршруты покрытия для всех роботов через Zenoh.

---

## Справочник пакетов

### `cslam` — Базовый Collaborative SLAM

Реализация разреженного децентрализованного C-SLAM. Три основных ROS 2-узла:

- **Loop Closure Detection**: Извлекает глобальные дескрипторы (ScanContext, NetVLAD, CosPlace) из сенсорных данных, обменивается ими с соседями через Broker (vertex-cover или простой диалог) и предлагает циклы замыкания при нахождении совпадений.
- **LiDAR/Stereo/RGBD Handler**: Генерирует ключевые кадры из потока сенсоров, прореживает облака точек, выполняет ICP-верификацию для подтверждённых циклов замыкания (по умолчанию >25 inliers).
- **Pose Graph Manager** (C++): Поддерживает децентрализованный граф поз с помощью GTSAM. Обрабатывает меж-роботные и внутри-роботные циклы замыкания. Периодически запускает PGO. Транслирует оптимизированные TF-фреймы.

**Конфигурация:** `src/cslam/config/cslam/example.yaml`

**Ключевые параметры:** `voxel_size`, `registration_min_inliers`, `max_nb_robots`, `descriptor_technique`, `vertex_cover_selection`

### `cslam_experiments` — Запуск экспериментов и конфигурация

Launch-файлы и YAML-конфиги для запуска Swarm-SLAM с различными сенсорами и датасетами:

- **Реальные роботы:** Ouster LiDAR, Intel RealSense RGB-D, OAK-D
- **Датасеты:** KITTI, KITTI-360, GRACO, S3E, M2DGR, RealRecon
- **Одометрия:** RTAB-Map (ICP, visual, RGB-D)
- **Zenoh bridge:** Меж-роботная коммуникация через `zenoh-bridge-ros2dds`

**Список разрешённых топиков Zenoh** (`config/zenoh/zenoh_cslam.json5`):
```
/cslam/.*, /r.*/tf.*, /r.*/odom, /r.*/pointcloud,
/r.*/cmd_vel, /r.*/darp/.*, /anomaly_detection/.*,
.*/heartbeat_checker/.*, /frontier/.*
```

### `cslam_visualization` — Визуализация в RViz2

Визуализация в реальном времени для мониторинга SLAP на базовой станции. Подписывается на топики C-SLAM через Zenoh и отображает графы поз, ключевые точки и облака точек в RViz2.

**Конфигурация:** `config/*.rviz` (LiDAR, RealSense, Stereo), `config/*.yaml`

### `frontier_exploration` — Автономное исследование на основе фронтиров

Ведёт постоянную карту занятости с настраиваемым разрешением (по умолчанию 0.1 м / 22×22 м). Конвейер исследования:

1. **Приём LiDAR** → Облако точек преобразуется в фрейм `robot0_map`
2. **Обновление карты** → Трассировка лучей (Bresenham) отмечает FREE на свободных клетках; точки препятствий — OCCUPIED
3. **Поиск фронтиров** → FREE клетки, граничащие с UNKNOWN
4. **Кластеризация** → Связные компоненты (≥10 клеток), проверка свободного пространства вокруг (запас 0.6 м)
5. **Выбор цели** → Ближайшая свободная клетка в каждом кластере, ограничена 5.0 м
6. **Координация** → Протокол резервирования: `exclusion_radius=3.0 м` предотвращает пересечение целей; штрафные оценки смещают роботов к незанятым фронтирам
7. **Достижимость** → `navigator.getPath()` проверяет глобальный план Nav2 перед отправкой цели
8. **Цикл мониторинга** → 1 Гц; таймаут 120 с; по завершении → перезапуск таймера исследования (5 с)

Когда все фронтиры исчерпаны, система уведомляет DARP bridge о запуске покрытия территории.

| Параметр | По умолч. | Описание |
|----------|-----------|----------|
| `grid_resolution` | 0.1 | Размер клетки карты (м) |
| `grid_width` / `grid_height` | 22.0 / 22.0 | Размеры карты (м) |
| `sensor_range` | 3.0 | Радиус интеграции LiDAR (м) |
| `exploration_period` | 5.0 | Интервал цикла исследования (с) |
| `frontier_timeout` | 120.0 | Таймаут на цель (с) |
| `dispersion_threshold` | 4.0 | Радиус штрафной зоны координатора (м) |
| `coordination_exclusion_radius` | 3.0 | Радиус исключения резервирования (м) |

### `darp_areas` — DARP разделение территории и маршруты покрытия

ROS 2-обёртка алгоритма DARP. На реальных роботах запускается в домене каждого робота; активен только экземпляр брокера. Получает объединённую карту занятости через сообщение `WakeUp`, строит растровую карту, запускает DARP для разделения территории между роботами и публикует маршруты покрытия `nav_msgs/Path`.

**Процесс:**
1. Получает `WakeUp` с целевым разрешением, расширением препятствий и отступом
2. Уменьшает разрешение карты фронтиров до разрешения DARP (0.5 м) через `cv2.INTER_AREA`
3. Применяет расширение препятствий (`cv2.dilate`, настраиваемый размер ядра)
4. Применяет отступ (100 = занятая граница)
5. Извлекает наибольший FREE связный компонент (доступная область)
6. Запускает алгоритм DARP (удвоенная сетка + STC) на назначенном участке
7. Публикует `/{ns}/darp/route` и `/{ns}/darp/area_markers`

| Параметр | По умолч. | Описание |
|----------|-----------|----------|
| `robot_count` | 1 | Количество роботов |
| `robot_prefix` | r | Префикс топиков робота |
| `default_frame_id` | robot0_map | Общая система координат |
| `default_padding` | 1.0 | Отступ карты (м) |
| `default_obstacle_dilation` | 1 | Расширение препятствий (клетки) |
| `expected_width` / `expected_height` | 220 / 220 | Ожидаемый размер сетки DARP (клетки) |

<div align="center">
  <img src="media/path.png" alt="Rviz: маршруты DARP, карта занятости и расширение препятствий" width="600"/>
  <p><em>Визуализация Rviz: маршруты DARP поверх общей карты занятости с расширением препятствий (obstacle dilation)</em></p>
</div>

### `nav_darp` — Стек навигации Nav2

Полный запуск Nav2 для каждого робота, включая:

- **Ноды Nav2:** controller_server, planner_server, smoother_server, behavior_server, BT navigator, lifecycle manager
- **Costmaps:** Локальная скользящего окна + глобальная статическая
- **BT Navigator:** Следование маршрутам DARP (`follow_path.xml`) или навигация к целям фронтиров (`navigate_to_pose.xml`)
- **DARP Path Follower:** Подписывается на `/{ns}/darp/route` и последовательно публикует цели
- **Odom-to-Map Publisher:** Слушает сообщения `PoseGraph` от C-SLAM и публикует TF `robot{X}_map → r{X}/odom`
- **Realtime Point Cloud:** Преобразует `r{X}/laser_frame` → `r{X}/pointcloud_real` для сенсорного ввода Nav2
- **Параметры Nav2:** Индивидуальные конфиги для каждого робота (`nav2_params_r0.yaml`, `nav2_params_r1.yaml`, `nav2_params_r2.yaml`)

### `diff_drive_robot` — Симуляция в Gazebo

Запускает N дифференциально-приводных роботов в симулированном мире. Каждый робот имеет:

- **Шасси:** Фиолетовый куб 0.3×0.3×0.15 м
- **Колёса:** Цилиндры радиусом 0.05 м
- **LiDAR:** 16-лучевой GPU ray, дальность 30 м, разрешение 1800×16, 10 Гц, тип `gpu_ray`
- **Плагин DiffDrive:** Расстояние между колёсами 0.35 м
- **OdometryPublisher:** 50 Гц, публикует `r{X}/odom` и TF

| Мир | Описание |
|-----|----------|
| `corridor.world` | Двойной квадрат (внешний 20×20 м, внутренний 10×10 м) с центральным столбом и проёмом во внутренней стене |
| `two_rooms.world` | Две соединённые комнаты, разделённые стеной с дверным проёмом |
| `world_pillar_forest.world` | Концентрические кольца столбов с L-образными препятствиями и узкими коридорами (симуляция леса или городского каньона) |

### `heartbeat_checker` — Мониторинг здоровья роботов

- Каждый робот публикует периодический heartbeat на топике `{ns}/heartbeat`
- Кросс-доменный чекер подписывается на heartbeats всех роботов
- Если heartbeat робота пропадает (настраиваемый таймаут), публикуется сообщение `LostRobots`
- DARP может переразделить территорию при потере робота

### `anomaly_detection` — Детекция препятствий в реальном времени

- Подписывается на облака точек LiDAR и карту занятости
- Для каждой точки LiDAR: проверяет, была ли клетка карты ранее FREE → помечает как **аномалия появления**
- Для каждого луча: проверяет, были ли клетки вдоль луча OCCUPIED → помечает как **аномалия исчезновения**
- Публикует сообщения `AnomalyDetected` с позой и типом
- Нода исследования реагирует:
  1. Устанавливает `anomaly_mask` (предотвращает повторный выбор области)
  2. Очищает затронутые клетки до UNKNOWN (повторное исследование)
  3. Публикует `anomaly_cleared`, когда все фронтиры исчерпаны

<div align="center">
  <img src="media/appearance_hit.png" alt="Rviz: обнаружение аномалии-появления" width="600"/>
  <p><em>Визуализация Rviz: попадание аномалии появления — новое препятствие обнаружено там, где карта ранее показывала свободное пространство</em></p>
</div>

---

## Коммуникация / Zenoh Bridge

Каждый робот работает на своём `ROS_DOMAIN_ID`. Zenoh bridge маршрутизирует определённые топики между доменами:

**Конфигурация bridge** (`config/zenoh/zenoh_cslam.json5`):
```json5
allow: {
  publishers: [
    "/cslam/.*", "/r.*/tf.*", "/r.*/odom", "/r.*/pointcloud",
    "/r.*/cmd_vel", "/r.*/darp/.*", "/anomaly_detection/.*",
    ".*/heartbeat_checker/.*", "/frontier/.*"
  ],
  subscribers: [ /* тот же список */ ]
}
```

Нода визуализации запускается на domain 100 со своим Zenoh bridge.

---

## Конфигурация

### Параметры CSLAM (`config/ouster_lidar.yaml`)

| Параметр | Значение |
|----------|----------|
| `sensor_type` | lidar |
| `voxel_size` | 0.3 |
| `registration_min_inliers` | 25 |
| `descriptor_technique` | scancontext / netvlad / cosplace |
| `vertex_cover_selection` | true |
| `enable_broadcast_tf_frames` | true |

### Параметры DARP Bridge

| Параметр | По умолч. | Описание |
|----------|-----------|----------|
| `robot_count` | 1 | Количество роботов |
| `frame_id` | robot0_map | Общая система координат |
| `padding` | 1.0 | Отступ карты (м) |
| `obstacle_dilation` | 1 | Расширение препятствий (клетки) |
| `expected_width/height` | 220 | Клетки сетки DARP |

### Параметры Exploration

| Параметр | По умолч. | Описание |
|----------|-----------|----------|
| `sensor_range` | 3.0 | Радиус интеграции LiDAR (м) |
| `grid_resolution` | 0.1 | Размер клетки карты (м) |
| `grid_width/grid_height` | 22.0 | Размеры карты (м) |
| `exploration_period` | 5.0 | Интервал цикла исследования (с) |
| `frontier_timeout` | 120.0 | Таймаут на цель (с) |

### Параметры Nav2 (`nav2_params_rX.yaml`)

Индивидуальные конфигурации costmap для каждого робота. `global_frame: robot{X}_map`. Выбор дерева поведения (follow_path vs navigate_to_pose).

---

## Лицензии сторонних компонентов

| Компонент | Лицензия | Авторские права |
|-----------|----------|-----------------|
| Swarm-SLAM core (`cslam`, `cslam_common_interfaces`, `cslam_experiments`) | MIT | 2022 Pierre-Yves Lajoie |
| `cslam_visualization` | BSD | Pierre-Yves Lajoie |
| Алгоритм DARP (`darp_areas/src/lib/`) | CC BY-NC 4.0 | Athanasios Kapoutsis et al. |
| `darp_areas` (ROS 2-обёртка) | MIT | — |
| `frontier_exploration` | MIT | — |
| `heartbeat_checker` | MIT | — |
| `anomaly_detection` | MIT | — |
| `nav_darp` | Apache 2.0 | — |

---

## Авторы

- **Pierre-Yves Lajoie** — Основной автор Swarm-SLAM (C-SLAM, эксперименты, визуализация)
- **Giovanni Beltrame** — Соавтор, руководитель MISTLab
- **Athanasios Kapoutsis et al.** — Оригинальный алгоритм DARP
- **MISTLab** (École Polytechnique de Montréal) — Исследовательская лаборатория, создавшая Swarm-SLAM

### Публикации

- P.-Y. Lajoie, B. Ramtoula, Y. Chang, L. Carlone, and G. Beltrame, "DOOR-SLAM: Distributed, Online, and Outlier Resilient SLAM for Robotic Teams," *IEEE Robotics and Automation Letters*, 2020.
- P.-Y. Lajoie and G. Beltrame, "Swarm-SLAM: Sparse Decentralized Collaborative Simultaneous Localization and Mapping Framework for Multi-Robot Systems," *arXiv preprint arXiv:2301.06230*, 2023.
- A. Kapoutsis, S. Chatzichristofis, and E. Kosmatopoulos, "DARP: Divide Areas Algorithm for Optimal Multi-Robot Coverage," *Journal of Intelligent & Robotic Systems*, 2017.

---

## Структура проекта

```
Swarm-Patrol/
├── src/
│   ├── cslam/                          # Базовый C-SLAM (submodule)
│   │   ├── cslam/                      # Python-ноды
│   │   └── cslam_core/                 # C++ pose graph manager
│   ├── cslam_common_interfaces/        # 23 пользовательских типа сообщений ROS 2
│   ├── cslam_experiments/              # Launch-файлы, конфиги, датасеты
│   │   ├── config/                     # YAML экспериментов + Zenoh JSON5
│   │   ├── launch/                     # Запуск экспериментов с роботами/датасетами
│   │   └── docker/                     # Dockerfile + makefile
│   ├── cslam_visualization/            # Нода визуализации RViz2
│   ├── darp_areas/                     # Нода-обёртка DARP
│   │   ├── darp_bridge_node.py         # ROS 2-обёртка
│   │   └── src/                        # Реализация алгоритма DARP
│   ├── diff_drive_robot/               # Симуляция Gazebo
│   │   ├── urdf/                       # XACRO модель робота
│   │   ├── worlds/                     # Миры симуляции
│   │   ├── launch/                     # Спавнер роботов
│   │   └── config/                     # Конфиг моста Gazebo
│   ├── frontier_exploration/           # Автономное исследование
│   │   ├── frontier_exploration/       # Python-ноды
│   │   │   ├── exploration_node.py     # Главная нода исследования
│   │   │   ├── frontier_finder.py      # Поиск и кластеризация фронтиров
│   │   │   └── coordinator.py          # Мульти-роботная координация
│   │   └── launch/                     # Launch-файлы
│   ├── heartbeat_checker/              # Мониторинг здоровья роботов
│   ├── nav_darp/                       # Интеграция Nav2
│   │   ├── launch/                     # Запуск Nav2 для каждого робота
│   │   ├── config/                     # YAML-параметры Nav2
│   │   └── behavior_trees/             # BT следования маршруту
│   └── anomaly_detection/              # Детекция препятствий в реальном времени
├── gtsam/                              # Библиотека GTSAM (сборка из исходников)
├── media/                              # Скриншоты и медиа
│   ├── path.png                        # Маршруты DARP + карта
│   └── appearance_hit.png              # Детекция аномалии появления
└── README.md                           # Этот файл (русский) 
└── README_en.md                        # Английская версия (English version)
```

## Docker

Готовое Docker-окружение для экспериментов с LiDAR:

```bash
cd src/cslam_experiments/docker
make build       # Собрать образ
make cpu_run     # Запустить контейнер
make attach      # Войти в контейнер
```

Внутри контейнера:
```bash
make swarmslam-lidar ROBOT_ID=0   # Запустить C-SLAM + Zenoh для робота 0
make viz                           # Запустить RViz2 на domain 100
```
