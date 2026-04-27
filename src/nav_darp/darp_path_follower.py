#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import Path
import time


class DARPPathFollower(Node):
    """
    Нода для одного робота.
    Принимает параметр ROBOT_ID (целое число) и формирует namespace = r{ROBOT_ID}.
    Подписывается на топик /<namespace>/darp/route и отправляет путь в Nav2.
    """

    def __init__(self):
        super().__init__('darp_path_follower')

        # --- Параметры ---
        self.declare_parameter('ROBOT_ID', 0)
        #self.declare_parameter('use_sim_time', False)

        robot_id = self.get_parameter('ROBOT_ID').get_parameter_value().integer_value

        # Namespace: r0, r1, ...
        self.namespace = f'r{robot_id}'

        # --- Nav2 navigator ---
        self.navigator = BasicNavigator(namespace=self.namespace)
        self.get_logger().info(f'Navigator инициализирован для namespace: "{self.namespace}"')

        # --- Подписка на путь ---
        topic_name = f'/{self.namespace}/darp/route'
        self.sub = self.create_subscription(Path, topic_name, self.path_callback, 10)
        self.get_logger().info(f'Ожидание пути на топике: {topic_name}')

        # --- Состояние ---
        self.is_navigating = False
        self.timer = None

    def path_callback(self, path_msg: Path):
        """Обработчик входящего пути."""
        if self.is_navigating:
            self.get_logger().warn('Робот уже выполняет путь. Новый путь проигнорирован.')
            return

        if len(path_msg.poses) == 0:
            self.get_logger().warn('Получен пустой путь. Игнорируем.')
            return

        self.get_logger().info(
            f'Получен путь из {len(path_msg.poses)} точек. '
            f'frame_id: {path_msg.header.frame_id}'
        )

        # Актуализируем позы и путь сам с нынешним временем
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for pose in path_msg.poses:
            pose.header.stamp = path_msg.header.stamp

        # Запускаем движение
        self.navigator.followPath(path_msg)
        self.is_navigating = True

        # Создаём таймер мониторинга (если ещё не создан)
        if self.timer is None:
            self.timer = self.create_timer(0.5, self.monitor_navigation)

    def monitor_navigation(self):
        """Периодически проверяет, завершена ли задача."""
        if not self.is_navigating:
            return

        if not self.navigator.isTaskComplete():
            return

        result = self.navigator.getResult()

        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('Путь пройден успешно!')
        elif result == TaskResult.CANCELED:
            self.get_logger().warn('Навигация была отменена.')
        elif result == TaskResult.FAILED:
            self.get_logger().error('Навигация провалена!')
        else:
            self.get_logger().info(f'Задача завершена с результатом: {result}')

        self.is_navigating = False

        # Останавливаем таймер
        if self.timer is not None:
            self.timer.cancel()
            self.timer = None


def main(args=None):
    rclpy.init(args=args)

    node = DARPPathFollower()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
