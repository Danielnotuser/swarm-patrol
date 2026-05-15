#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from builtin_interfaces.msg import Time
from lifecycle_msgs.srv import GetState
from rosgraph_msgs.msg import Clock
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import tf2_ros
import tf2_geometry_msgs


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
        self.get_logger().info(f'Waiting for path on: {topic_name}')

        self.is_navigating = False
        self.timer = None

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

    def path_callback(self, path_msg: Path):
        if self.is_navigating:
            self.get_logger().warn('Robot already navigating. New path ignored.')
            return

        if len(path_msg.poses) == 0:
            self.get_logger().warn('Empty path. Ignored.')
            return

        self.get_logger().info(
            f'Received path with {len(path_msg.poses)} poses in frame "{path_msg.header.frame_id}"')

        stamp = self._last_clock

        if path_msg.header.frame_id != self.global_frame:
            self.get_logger().warn(f'Path frame "{path_msg.header.frame_id}" != "{self.global_frame}", transforming')
            for pose in path_msg.poses:
                pose.header.stamp = stamp
                transformed = self._transform_pose(
                    pose, path_msg.header.frame_id)
                pose.pose = transformed.pose
                pose.header.frame_id = self.global_frame
                pose.header.stamp = stamp
            path_msg.header.frame_id = self.global_frame

        path_msg.header.stamp = stamp

        self.get_logger().info(f"First pose in {self.global_frame}: {path_msg.poses[0].pose}")

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
            self.get_logger().info('Path completed successfully!')
        elif result == TaskResult.CANCELED:
            self.get_logger().warn('Navigation cancelled.')
        elif result == TaskResult.FAILED:
            self.get_logger().error('Navigation failed!')
        else:
            self.get_logger().info(f'Task result: {result}')

        self.is_navigating = False
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
