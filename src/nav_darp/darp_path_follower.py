#!/usr/bin/env python3

import time
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from builtin_interfaces.msg import Time
from lifecycle_msgs.srv import GetState
from rosgraph_msgs.msg import Clock
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray
import tf2_ros
import tf2_geometry_msgs

from anomaly_detection.msg import AnomalyDetected


class DARPPathFollower(Node):
    def __init__(self):
        super().__init__('darp_path_follower')

        self.declare_parameter('robot_id', 0)
        self.declare_parameter("global_frame", "robot0_map")

        self.robot_id = self.get_parameter('robot_id').get_parameter_value().integer_value
        self.global_frame = str(self.get_parameter("global_frame").value)

        self.namespace = f'r{self.robot_id}'
        self._last_clock = Time()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_subscription(Clock, f'/{self.namespace}/clock', self._clock_cb, 10)

        self.navigator = BasicNavigator(namespace=self.namespace)
        self._wait_for_lifecycle_nodes()
        self.get_logger().info('Nav2 is ready.')

        topic_name = f'/{self.namespace}/darp/route'
        qos = QoSProfile(depth=10)
        self.sub = self.create_subscription(Path, topic_name, self.path_callback, qos)
        self.anomaly_sub = self.create_subscription(AnomalyDetected, '/anomaly_detection/anomaly', self._anomaly_cb, 10)
        self._frontier_status_sub = self.create_subscription(
            Float32MultiArray, '/frontier/frontier_status',
            self._frontier_status_cb, 10
        )
        self.get_logger().info(f'Listening for DARP routes on: {topic_name}')

        self._frontier_done = False

        self.is_navigating = False
        self._current_path_len = 0
        self._current_path_hash = None
        self.timer = None
        self._cancel_pending = False

    def _wait_for_lifecycle_nodes(self) -> None:
        nodes = ['controller_server', 'smoother_server', 'velocity_smoother']
        for node_name in nodes:
            node_service = f'/{self.namespace}/{node_name}/get_state'
            self.get_logger().info(f'Waiting for {node_name}...')
            state_client = self.create_client(GetState, node_service)
            while not state_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'{node_service} not available, waiting...')
            req = GetState.Request()
            state = 'unknown'
            while state != 'active':
                future = state_client.call_async(req)
                rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
                if future.result() is not None:
                    state = future.result().current_state.label
                if state != 'active':
                    time.sleep(2)
            self.get_logger().info(f'{node_name} is active.')

        self.get_logger().info('All lifecycle nodes are active.')
        self.get_logger().info('Waiting 5 seconds for system stabilization...')
        time.sleep(5)
        self.get_logger().info('System ready.')

    def _clock_cb(self, msg: Clock) -> None:
        self._last_clock = msg.clock

    def _transform_pose(self, pose: PoseStamped, from_frame: str) -> PoseStamped:
        try:
            latest = rclpy.time.Time(seconds=0)
            transform = self.tf_buffer.lookup_transform(self.global_frame, from_frame, latest)
            result = tf2_geometry_msgs.do_transform_pose(pose, transform)
            result.header.stamp = self._last_clock
            return result
        except Exception as e:
            self.get_logger().warn(f'Transform failed: {e}')
            return pose

    def _get_robot_pose(self) -> tuple:
        try:
            t = self.tf_buffer.lookup_transform(
                self.global_frame, f'{self.namespace}/base_link',
                rclpy.time.Time(seconds=0)
            )
            return (t.transform.translation.x, t.transform.translation.y)
        except Exception:
            return (0.0, 0.0)

    def _path_hash(self, path: Path) -> int:
        h = len(path.poses)
        for p in path.poses:
            h ^= (int(p.pose.position.x * 100) & 0xFFFF) << (h % 16)
            h ^= (int(p.pose.position.y * 100) & 0xFFFF) << ((h + 8) % 16)
        return h

    def _find_nearest_pose_idx(self, path: Path, robot_x: float, robot_y: float) -> int:
        min_dist = float('inf')
        best_idx = 0
        for i, pose in enumerate(path.poses):
            d = math.hypot(pose.pose.position.x - robot_x, pose.pose.position.y - robot_y)
            if d < min_dist:
                min_dist = d
                best_idx = i
        return best_idx

    def _frontier_status_cb(self, msg: Float32MultiArray) -> None:
        if len(msg.data) < 2 or int(msg.data[0]) != self.robot_id:
            return
        was = self._frontier_done
        self._frontier_done = (msg.data[1] == 0.0)
        if was != self._frontier_done:
            self.get_logger().info(f'Frontier done? {self._frontier_done}')

    def path_callback(self, path_msg: Path):
        if len(path_msg.poses) == 0:
            self.get_logger().warn('Empty DARP path. Ignored.')
            return

        if not self._frontier_done:
            self.get_logger().info('Frontier not done yet, ignoring DARP path')
            return

        path_hash = self._path_hash(path_msg)
        if path_hash == self._current_path_hash:
            return

        stamp = self._last_clock

        if path_msg.header.frame_id != self.global_frame:
            self.get_logger().warn(f'Path frame "{path_msg.header.frame_id}" != "{self.global_frame}", transforming')
            for pose in path_msg.poses:
                pose.header.stamp = stamp
                transformed = self._transform_pose(pose, path_msg.header.frame_id)
                pose.pose = transformed.pose
                pose.header.frame_id = self.global_frame
                pose.header.stamp = stamp
            path_msg.header.frame_id = self.global_frame

        path_msg.header.stamp = stamp
        self._current_path_hash = path_hash
        self._current_path_len = len(path_msg.poses)
        self.get_logger().info(f'Received DARP path with {len(path_msg.poses)} poses (hash={path_hash})')

        if self.is_navigating:
            self.get_logger().warn('Currently navigating, canceling current task to follow new DARP path.')
            self.navigator.cancelTask()
            self._cancel_pending = True
            self.is_navigating = False
            if self.timer:
                self.timer.cancel()
                self.timer = None

        self._follow_path(path_msg)

    def _follow_path(self, path_msg: Path) -> None:
        robot_x, robot_y = self._get_robot_pose()
        nearest_idx = self._find_nearest_pose_idx(path_msg, robot_x, robot_y)

        if nearest_idx > 0:
            self.get_logger().info(f'Starting from pose {nearest_idx}/{len(path_msg.poses)-1} (nearest to robot at {robot_x:.2f}, {robot_y:.2f})')
            reordered = Path()
            reordered.header = path_msg.header
            reordered.poses = list(path_msg.poses[nearest_idx:]) + list(path_msg.poses[:nearest_idx])
            path_msg = reordered

        self.navigator.followPath(path_msg)
        self.is_navigating = True

        if self.timer is None:
            self.timer = self.create_timer(0.5, self.monitor_navigation)

    def monitor_navigation(self):
        if not self.is_navigating:
            return
        if not self.navigator.isTaskComplete():
            return

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('DARP path completed successfully!')
        elif result == TaskResult.CANCELED:
            if self._cancel_pending:
                self.get_logger().info('Navigation canceled (pending new path).')
                self._cancel_pending = False
            else:
                self.get_logger().warn('Navigation canceled.')
        elif result == TaskResult.FAILED:
            self.get_logger().error('Navigation failed!')
        else:
            self.get_logger().info(f'Task result: {result}')

        self.is_navigating = False
        self._current_path_hash = None
        if self.timer is not None:
            self.timer.cancel()
            self.timer = None

    def _anomaly_cb(self, msg: AnomalyDetected) -> None:
        if msg.robot_id != self.robot_id:
            return
        if not self.is_navigating:
            return

        self.get_logger().warn(
            f'Anomaly detected for robot {msg.robot_id} at pose ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f}), '
            'canceling DARP navigation.'
        )
        self.navigator.cancelTask()
        self.is_navigating = False
        self._current_path_hash = None
        self._frontier_done = False
        if self.timer is not None:
            self.timer.cancel()
            self.timer = None


def main(args=None):
    rclpy.init(args=args)
    node = DARPPathFollower()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
