#!/usr/bin/env python3

import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformException
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import Path
from copy import deepcopy
import time

def quat_to_rot_matrix(q: Quaternion) -> np.ndarray:
    x, y, z, w = q.x, q.y, q.z, q.w
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z

    return np.array(
        [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ],
        dtype=np.float64,
    )

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

        self.robot_id = self.get_parameter('ROBOT_ID').get_parameter_value().integer_value

        # Namespace: r0, r1, ...
        self.namespace = f'r{self.robot_id}'

        # --- Nav2 navigator ---
        self.navigator = BasicNavigator(namespace=self.namespace)
        self.get_logger().info(f'Navigator инициализирован для namespace: "{self.namespace}"')

        # --- Подписка на путь ---
        topic_name = f'/{self.namespace}/darp/route'
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.sub = self.create_subscription(Path, topic_name, self.path_callback, qos)
        self.get_logger().info(f'Ожидание пути на топике: {topic_name}')

        # --- Состояние ---
        self.is_navigating = False
        self.timer = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    # ------------------------ TF helpers ------------------------

    def _lookup_transform_matrix(self, target_frame: str, source_frame: str) -> np.ndarray:
        if not source_frame or source_frame == target_frame:
            return np.eye(4, dtype=np.float64)

        tf = self.tf_buffer.lookup_transform(target_frame, source_frame, Time(seconds=0))
        R = quat_to_rot_matrix(tf.transform.rotation)
        t = tf.transform.translation

        T = np.eye(4, dtype=np.float64)
        T[:3, :3] = R
        T[0, 3] = float(t.x)
        T[1, 3] = float(t.y)
        T[2, 3] = float(t.z)
        return T

    def _try_transform_pose(self, pose, source_frame, target_frame) -> Pose:
        try:
            tf = self.tf_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().warn(f"TF not ready yet: {source_frame} -> {target_frame}: {ex}")
            return None

        T = self._lookup_transform_matrix(target_frame, source_frame)
        v = np.array([pose.position.x, pose.position.y, pose.position.z, 1.0], dtype=np.float64)
        w = T @ v

        out = deepcopy(pose)
        out.position.x = float(w[0])
        out.position.y = float(w[1])
        out.position.z = float(w[2])
        return out

    def _transform_pose_to_frame(self, pose: Pose, source_frame: str, target_frame: str) -> Pose:
        if not source_frame or source_frame == target_frame:
            return deepcopy(pose)

        while not self.tf_buffer.can_transform(target_frame, source_frame, Time(seconds=0), timeout=rclpy.duration.Duration(seconds=5.0)):
            self.get_logger().warn(
                f"TF not ready yet: {source_frame} -> {target_frame}"
            )

        T = self._lookup_transform_matrix(target_frame, source_frame)
        v = np.array([pose.position.x, pose.position.y, pose.position.z, 1.0], dtype=np.float64)
        w = T @ v

        out = deepcopy(pose)
        out.position.x = float(w[0])
        out.position.y = float(w[1])
        out.position.z = float(w[2])
        return out

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

        #for pose in path_msg.poses:
        #    pose.header.stamp = path_msg.header.stamp

        self.get_logger().info(f"Start pose in robot0_map frame --> {path_msg.poses[0].pose}")

        transformed_pose = self._try_transform_pose(
            path_msg.poses[0].pose,
            "robot0_map",
            f"robot{self.robot_id}_current_pose"
        )

        if transformed_pose is None:
            self.get_logger().warn("TF ещё не готов. Путь пока не отправляю.")
            return

        self.get_logger().info(f"Start pose in robot{self.robot_id}_current_pose frame --> {transformed_pose}")

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
