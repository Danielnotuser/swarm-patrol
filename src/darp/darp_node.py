import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
import numpy as np
import sys

# Импорт оригинальной реализации DARP
try:
    from darp import DARP
except ImportError:
    print("Ошибка: Модуль 'darp' не найден.")
    print("Убедитесь, что пакет DARP установлен в вашем окружении Python.")
    sys.exit(1)

class DARPNode(Node):
    def __init__(self):
        super().__init__('darp_node')

        # --- Параметры ноды ---
        self.declare_parameter('map_topic', '/darp/map')
        self.declare_parameter('robot_initial_positions', [0, 3, 9]) # Номера ячеек
        self.declare_parameter('portions', [0.33, 0.33, 0.34]) # Доли площадей
        self.declare_parameter('not_equal_portions', False)
        self.declare_parameter('max_iter', 80000)
        self.declare_parameter('cc_variation', 0.01)
        self.declare_parameter('random_level', 0.0001)
        self.declare_parameter('dcells', 2)
        self.declare_parameter('importance', False)

        # --- Получение параметров ---
        self.map_topic = self.get_parameter('map_topic').value
        self.initial_pos = self.get_parameter('robot_initial_positions').value
        self.portions = self.get_parameter('portions').value
        self.not_equal = self.get_parameter('not_equal_portions').value
        self.max_iter = self.get_parameter('max_iter').value
        self.cc_variation = self.get_parameter('cc_variation').value
        self.random_level = self.get_parameter('random_level').value
        self.dcells = self.get_parameter('dcells').value
        self.importance = self.get_parameter('importance').value

        # --- Подписка на карту ---
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self.map_callback,
            10)
        self.get_logger().info(f"Подписан на топик: {self.map_topic}")

        # --- Хранилище для паблишеров (создаются динамически) ---
        self.publishers = {}
        self.overall_pub = self.create_publisher(OccupancyGrid, '/darp/areas_divided', 10)

    def map_callback(self, msg):
        """Вызывается при получении новой карты."""
        self.get_logger().info("Получена карта. Запуск DARP...")

        # 1. Преобразование OccupancyGrid в формат DARP
        grid_env, initial_positions_cells = self.convert_map_to_darp_format(msg)

        # 2. Проверка корректности параметров
        if not self.validate_parameters(grid_env, initial_positions_cells):
            return

        # 3. Создание и запуск экземпляра DARP
        darp = DARP(
            nx=msg.info.height, ny=msg.info.width,
            notEqualPortions=self.not_equal,
            given_initial_positions=initial_positions_cells,
            given_portions=self.portions,
            obstacles_positions=self.extract_obstacles(grid_env),
            visualization=False, # В ROS2 визуализация через RViz2
            MaxIter=self.max_iter,
            CCvariation=self.cc_variation,
            randomLevel=self.random_level,
            dcells=self.dcells,
            importance=self.importance
        )

        # 4. Запуск деления
        success, iterations = darp.divideRegions()
        if not success:
            self.get_logger().error("DARP не смог найти решение!")
            return

        self.get_logger().info(f"DARP завершен успешно за {iterations} итераций.")

        # 5. Публикация результатов
        self.publish_results(darp, msg)

    def convert_map_to_darp_format(self, msg):
        """
        Преобразует сообщение OccupancyGrid в матрицу для DARP.
        Возвращает:
            grid_env: numpy-массив (height x width) со значениями:
                       -2 для препятствий, -1 для свободных ячеек.
            initial_positions_cells: список номеров ячеек для роботов.
        """
        data = np.array(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
        grid_env = np.full_like(data, -1, dtype=np.int8)
        grid_env[data > 65] = -2 # Значения OccupancyGrid > 65 считаем препятствием

        # Номера ячеек для начальных позиций уже переданы как параметры
        initial_positions_cells = self.initial_pos
        return grid_env, initial_positions_cells

    def extract_obstacles(self, grid_env):
        """Возвращает список номеров ячеек с препятствиями."""
        obstacle_indices = np.where(grid_env == -2)
        rows, cols = obstacle_indices
        return (rows * grid_env.shape[1] + cols).tolist()

    def validate_parameters(self, grid_env, initial_positions_cells):
        """Проверяет, что параметры корректны."""
        # Проверка, что роботы не находятся на препятствиях
        for pos_cell in initial_positions_cells:
            row = pos_cell // grid_env.shape[1]
            col = pos_cell % grid_env.shape[1]
            if grid_env[row, col] == -2:
                self.get_logger().error(f"Робот в позиции {pos_cell} находится на препятствии!")
                return False
        # Проверка суммы долей
        if abs(sum(self.portions) - 1.0) > 1e-4:
            self.get_logger().error("Сумма долей не равна 1.0!")
            return False
        return True

    def publish_results(self, darp, map_msg):
        """
        Публикует результаты деления.
        - Для каждого робота публикуется бинарная карта его зоны в /rX/darp/area
        - Общая карта с ID роботов публикуется в /darp/areas_divided
        """
        num_robots = darp.droneNo
        assignment_matrix = darp.A # Матрица размера (height x width) с ID роботов

        # 1. Публикация индивидуальных зон
        for r in range(num_robots):
            topic_name = f'/r{r}/darp/area'

            # Создаем паблишер, если его еще нет
            if r not in self.publishers:
                self.publishers[r] = self.create_publisher(OccupancyGrid, topic_name, 10)
                self.get_logger().info(f"Создан паблишер для {topic_name}")

            # Создаем сообщение
            grid_msg = OccupancyGrid()
            grid_msg.header = Header()
            grid_msg.header.stamp = self.get_clock().now().to_msg()
            grid_msg.header.frame_id = map_msg.header.frame_id
            grid_msg.info = map_msg.info

            # Бинарная маска: 100 для ячеек робота, 0 для остальных
            binary_data = np.zeros_like(assignment_matrix, dtype=np.int8)
            binary_data[assignment_matrix == r] = 100
            grid_msg.data = binary_data.flatten().tolist()

            self.publishers[r].publish(grid_msg)

        # 2. Публикация общей карты
        overall_msg = OccupancyGrid()
        overall_msg.header = Header()
        overall_msg.header.stamp = self.get_clock().now().to_msg()
        overall_msg.header.frame_id = map_msg.header.frame_id
        overall_msg.info = map_msg.info

        # Масштабируем ID роботов в диапазон [0, 100] для OccupancyGrid
        if num_robots > 1:
            scaled_data = (assignment_matrix / (num_robots - 1) * 100).astype(np.int8)
        else:
            scaled_data = assignment_matrix.astype(np.int8)
        overall_msg.data = scaled_data.flatten().tolist()

        self.overall_pub.publish(overall_msg)
        self.get_logger().info("Результаты опубликованы.")

def main(args=None):
    rclpy.init(args=args)
    node = DARPNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()