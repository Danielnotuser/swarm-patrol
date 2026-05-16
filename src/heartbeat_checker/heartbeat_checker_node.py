#!/usr/bin/env python3
import threading
from collections import defaultdict
from typing import Dict, List, Optional

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from std_msgs.msg import UInt32

from heartbeat_checker.msg import LostRobots
from darp_areas.msg import WakeUp



class HeartbeatCheckerNode(Node):
    def __init__(self) -> None:
        super().__init__("heartbeat_checker_node")
        self.cb_group = ReentrantCallbackGroup()

        self.declare_parameter("robot_prefix", "r")
        self.declare_parameter("robot_id", 0)
        self.declare_parameter("robot_count", 2)
        self.declare_parameter("heartbeat_timeout", 5.0)
        self.declare_parameter("wake_up_delay", 3.0)
        self.declare_parameter("check_period", 0.5)
        self.declare_parameter("heartbeat_publish_period", 1.0)
        self.declare_parameter("heartbeat_lost_topic", "/heartbeat_checker/lost")
        self.declare_parameter("heartbeat_topic", "heartbeat_checker/heartbeat")

        self.robot_prefix = str(self.get_parameter("robot_prefix").value)
        self.robot_id = int(self.get_parameter("robot_id").value)
        self.robot_count = int(self.get_parameter("robot_count").value)
        self.heartbeat_timeout = float(self.get_parameter("heartbeat_timeout").value)
        self.wake_up_delay = float(self.get_parameter("wake_up_delay").value)
        self.check_period = float(self.get_parameter("check_period").value)
        self.heartbeat_publish_period = float(self.get_parameter("heartbeat_publish_period").value)
        self.heartbeat_lost_topic = str(self.get_parameter("heartbeat_lost_topic").value)
        self.heartbeat_topic = str(self.get_parameter("heartbeat_topic").value)

        self.namespace = f"{self.robot_prefix}{self.robot_id}"

        self.heartbeat_pub = self.create_publisher(
            UInt32, f"/{self.namespace}/{self.heartbeat_topic}", 10)
        self.create_timer(self.heartbeat_publish_period, self._publish_heartbeat)

        # --- Per-robot heartbeat subscriptions ---
        self.last_seen: Dict[int, float] = defaultdict(lambda: float("-inf"))
        self.lock = threading.Lock()

        for rid in range(self.robot_count):
            if rid == self.robot_id:
                continue
            topic = f"/{self.robot_prefix}{rid}/{self.heartbeat_topic}"
            self.create_subscription(
                UInt32, topic, lambda msg, robid=rid: self._heartbeat_cb(robid),
                10, callback_group=self.cb_group)

        self.get_logger().info(
            f"Monitoring {self.robot_count} robots via /r*/{self.heartbeat_topic}")

        self.get_logger().info(
            f"Robot topics: /{self.namespace}/{self.heartbeat_topic}")

        # --- Periodic check ---
        self.create_timer(self.check_period, self._check_heartbeats)

        # --- Lost publisher ---
        self.lost_pub = self.create_publisher(
            LostRobots, self.heartbeat_lost_topic, 10)

        # --- Wake-up publisher ---
        self.wake_up_pub = self.create_publisher(
            WakeUp, "/darp/wake_up", 10)

        # --- Dedup state ---
        self.last_lost_id: Optional[int] = None
        self.last_lost_remaining: Optional[List[int]] = None
        self.wake_up_pending = False
        self.wake_up_timer = None

    # --------------------------------------------------------------

    def _publish_heartbeat(self) -> None:
        msg = UInt32()
        msg.data = self.robot_id
        self.heartbeat_pub.publish(msg)

    def _heartbeat_cb(self, rid: int) -> None:
        with self.lock:
            if self.last_seen[rid] == float("-inf"):
                self.get_logger().info(f"First heartbeat from robot {rid}")
            self.last_seen[rid] = self.get_clock().now().nanoseconds / 1e9

    def _check_heartbeats(self) -> None:
        now = self.get_clock().now().nanoseconds / 1e9

        with self.lock:
            lost = [rid for rid in range(self.robot_count)
                    if self.last_seen[rid] != float("-inf")
                    and now - self.last_seen[rid] > self.heartbeat_timeout]

        if not lost:
            return

        lost_id = lost[0]
        remaining = [rid for rid in range(self.robot_count) if rid not in lost]

        if self.last_lost_id == lost_id and self.last_lost_remaining == remaining:
            return

        self.last_lost_id = lost_id
        self.last_lost_remaining = remaining

        self.get_logger().warn(
            f"Robot {lost_id} lost (timeout). Remaining: {remaining}")

        msg = LostRobots()
        msg.lost_robot_id = lost_id
        msg.remaining_robot_ids = remaining
        self.lost_pub.publish(msg)

        if not self.wake_up_pending:
            self.wake_up_pending = True
            self._schedule_wake_up(remaining)

    def _schedule_wake_up(self, remaining: List[int]) -> None:
        def _do_call():
            self.wake_up_pending = False
            if self.wake_up_timer is not None:
                self.wake_up_timer.cancel()
                self.wake_up_timer = None
            self._call_wake_up(remaining)

        self.wake_up_timer = self.create_timer(self.wake_up_delay, _do_call)

    def _call_wake_up(self, remaining: List[int]) -> None:
        msg = WakeUp()
        msg.resolution = 0.5
        msg.padding = 0.0
        msg.obstacle_dilation = 1
        msg.use_equal_portions = True
        msg.portions = []
        msg.active_robot_ids = remaining
        self.wake_up_pub.publish(msg)
        self.get_logger().info(f"Wake-up requested for robots: {remaining}")


def main(args=None) -> None:
    import signal
    signal.signal(signal.SIGINT, signal.default_int_handler)
    rclpy.init(args=args)
    node = HeartbeatCheckerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
