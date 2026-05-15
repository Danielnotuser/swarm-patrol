#!/usr/bin/env python3

from copy import deepcopy

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from tf2_msgs.msg import TFMessage
import tf2_ros


TF_TIMEOUT = Duration(seconds=2.0)
STARTUP_TIMEOUT = Duration(seconds=30.0)


class RealtimePointcloud(Node):
    def __init__(self) -> None:
        super().__init__("realtime_pointcloud")
        self.cb_group = ReentrantCallbackGroup()

        self.declare_parameter("robot_prefix", "r")
        self.declare_parameter("robot_id", 0)
        self.declare_parameter("input_pointcloud_topic", "pointcloud")
        self.declare_parameter("output_pointcloud_topic", "pointcloud_real")
        self.declare_parameter("target_frame", "r0/laser_frame")
        self.declare_parameter("global_frame", "robot0_map")
        self.declare_parameter("tf_topic", "/tf")
        self.declare_parameter("tf_static_topic", "/r0/tf_static")

        self.robot_prefix = str(self.get_parameter("robot_prefix").value)
        self.robot_id = str(self.get_parameter('robot_id').value)
        self.input_pointcloud_topic = str(self.get_parameter("input_pointcloud_topic").value)
        self.output_pointcloud_topic = str(self.get_parameter("output_pointcloud_topic").value)
        self.target_frame = str(self.get_parameter("target_frame").value)
        self.global_frame = str(self.get_parameter("global_frame").value)
        self.dynamic_topic = str(self.get_parameter("tf_topic").value)
        self.static_topic = str(self.get_parameter("tf_static_topic").value)



        self.tf_buffer = tf2_ros.Buffer()

        tf_qos = QoSProfile(
            depth=100,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )
        static_qos = QoSProfile(
            depth=100,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )

        self.create_subscription(TFMessage, self.dynamic_topic, self._tf_cb, tf_qos, callback_group=self.cb_group)
        self.create_subscription(TFMessage, self.static_topic, self._tf_static_cb, static_qos, callback_group=self.cb_group)

        deadline = self.get_clock().now() + STARTUP_TIMEOUT
        last_log = self.get_clock().now()
        self.get_logger().info(
            f"Waiting for tf chain '{self.global_frame} -> {self.target_frame}'...")
        while not self.tf_buffer.can_transform(
            self.global_frame, self.target_frame,
            Time(), timeout=Duration(seconds=0.1)):
            now = self.get_clock().now()
            if now > deadline:
                frames = self.tf_buffer.all_frames_as_yaml()
                self.get_logger().error(
                    f"Timed out. Frames in buffer:\n{frames}")
                raise RuntimeError("Startup timeout waiting for tf chain")
            if now - last_log > Duration(seconds=5.0):
                self.get_logger().info(
                    f"Still waiting... frames:\n{self.tf_buffer.all_frames_as_yaml()}")
                last_log = now
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info(f"Tf chain '{self.global_frame} -> {self.target_frame}' is ready.")

        cloud_input_topic = f"/{self.robot_prefix}{self.robot_id}/{self.input_pointcloud_topic}"
        cloud_output_topic = f"/{self.robot_prefix}{self.robot_id}/{self.output_pointcloud_topic}"

        cloud_input_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )

        self.pointcloud_sub = self.create_subscription(
            PointCloud2, cloud_input_topic, self._cloud_cb, cloud_input_qos, callback_group=self.cb_group)
        self.pointcloud_pub = self.create_publisher(PointCloud2, cloud_output_topic, 10)

        self.get_logger().info(
            f"Started. {cloud_input_topic} -> {cloud_output_topic}, "
            f"global: {self.global_frame}, target: {self.target_frame}")

    def _tf_cb(self, msg: TFMessage) -> None:
        for transform in msg.transforms:
            self.tf_buffer.set_transform(transform, 'default_authority')

    def _tf_static_cb(self, msg: TFMessage) -> None:
        for transform in msg.transforms:
            self.tf_buffer.set_transform_static(transform, 'default_authority')

    def _cloud_cb(self, msg: PointCloud2) -> None:
        out = deepcopy(msg)
        out.header.frame_id = self.target_frame

        try:
            t = self.tf_buffer.lookup_transform(
                self.global_frame, self.target_frame,
                Time(), timeout=TF_TIMEOUT)
            out.header.stamp = t.header.stamp
        except Exception as e:
            self.get_logger().warn(
                f"Failed to lookup tf {self.global_frame} -> {self.target_frame}: {e}",
                throttle_duration_sec=5.0)
            return

        self.pointcloud_pub.publish(out)


def main() -> None:
    rclpy.init()
    node = RealtimePointcloud()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
