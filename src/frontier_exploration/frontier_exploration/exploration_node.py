#!/usr/bin/env python3

import signal
import time
from typing import Dict, List, Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.duration import Duration
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Int32, UInt8MultiArray
from lifecycle_msgs.srv import GetState
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid, Path
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from tf2_msgs.msg import TFMessage
import tf2_ros

from cslam_common_interfaces.msg import PoseGraph
from darp_areas.msg import WakeUp
from anomaly_detection.msg import AnomalyDetected

from frontier_exploration.frontier_finder import FrontierFinder
from frontier_exploration.coordinator import Coordinator

STARTUP_TIMEOUT = Duration(seconds=30.0)


class ExplorationState:
    IDLE = "idle"
    EXPLORING = "exploring"
    DONE = "done"


class ExplorationNode(Node):
    def __init__(self):
        super().__init__('exploration_node')

        self.declare_parameter('robot_id', 0)
        self.declare_parameter('robot_count', 2)
        self.declare_parameter('grid_resolution', 0.1)
        self.declare_parameter('grid_size', 40.0)
        self.declare_parameter('z_min', 0.0)
        self.declare_parameter('cluster_dist', 1.5)
        self.declare_parameter('min_passage_width', 1.0)
        self.declare_parameter('reservation_ttl', 30.0)
        self.declare_parameter('frontier_timeout', 120.0)
        self.declare_parameter('exploration_period', 5.0)
        self.declare_parameter('dispersion_threshold', 4.0)
        self.declare_parameter('coordination_exclusion_radius', 3.0)
        self.declare_parameter('base_frame', 'laser_frame')
        self.declare_parameter('tf_topic', '/tf')
        self.declare_parameter('tf_static_topic', '/r0/tf_static')

        self.robot_id = self.get_parameter('robot_id').get_parameter_value().integer_value
        self.robot_count = self.get_parameter('robot_count').get_parameter_value().integer_value
        self.grid_resolution = float(self.get_parameter('grid_resolution').value)
        self.grid_size = float(self.get_parameter('grid_size').value)
        self.z_min = float(self.get_parameter('z_min').value)
        self.cluster_dist = float(self.get_parameter('cluster_dist').value)
        self.min_passage_width = float(self.get_parameter('min_passage_width').value)
        self.reservation_ttl = float(self.get_parameter('reservation_ttl').value)
        self.frontier_timeout = float(self.get_parameter('frontier_timeout').value)
        self.exploration_period = float(self.get_parameter('exploration_period').value)
        self.dispersion_threshold = float(self.get_parameter('dispersion_threshold').value)
        self.coordination_exclusion_radius = float(self.get_parameter('coordination_exclusion_radius').value)
        self.input_base_frame = str(self.get_parameter('base_frame').value)
        self.tf_topic = str(self.get_parameter('tf_topic').value)
        self.tf_static_topic = str(self.get_parameter('tf_static_topic').value)

        self.namespace = f'r{self.robot_id}'
        self.global_frame = 'robot0_map'
        self.nav_frame = f'robot{self.robot_id}_map'
        self.base_frame = f'{self.namespace}/{self.input_base_frame}'

        self._state = ExplorationState.IDLE
        self._current_goal: Optional[Tuple[float, float]] = None
        self._goal_start_time: Optional[float] = None
        self._exploration_timer = None
        self._monitor_timer = None
        self._anomaly_mask: Optional[np.ndarray] = None
        self._anomaly_cleared_sent = False

        self.tf_buffer = tf2_ros.Buffer()

        tf_qos = QoSProfile(depth=100, durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST)
        static_qos = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(TFMessage, self.tf_topic, self._tf_cb, tf_qos)
        self.create_subscription(TFMessage, self.tf_static_topic, self._tf_static_cb, static_qos)

        self.get_logger().info(f'Waiting for tf chain {self.nav_frame} -> {self.base_frame}...')
        deadline = self.get_clock().now() + STARTUP_TIMEOUT
        last_log = self.get_clock().now()
        while not self.tf_buffer.can_transform(self.nav_frame, self.base_frame, Time(), timeout=Duration(seconds=0.1)):
            now = self.get_clock().now()
            if now > deadline:
                frames = self.tf_buffer.all_frames_as_yaml()
                self.get_logger().error(f'Timed out waiting for tf chain. Frames in buffer:\n{frames}')
                raise RuntimeError('Startup timeout waiting for tf chain')
            if now - last_log > Duration(seconds=5.0):
                self.get_logger().warn(f'Still waiting for tf chain... ({self.nav_frame} -> {self.base_frame})')
                last_log = now
            rclpy.spin_once(self, timeout_sec=0.1)

        t = TransformStamped()
        t.header.stamp = Time()
        t.header.frame_id = self.nav_frame
        t.child_frame_id = self.global_frame
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.w = 1.0
        self.tf_buffer.set_transform_static(t, 'exploration_node')
        msg = TFMessage()
        msg.transforms = [t]
        self.create_publisher(TFMessage, self.tf_static_topic, static_qos)
        self.get_logger().info(f'Static transform {self.nav_frame} -> {self.global_frame} injected')

        self.navigator = BasicNavigator(namespace=self.namespace)
        self._wait_for_lifecycle_nodes()
        self.get_logger().info('Nav2 is ready.')

        self.frontier_finder = FrontierFinder(
            resolution=self.grid_resolution,
            grid_size=self.grid_size,
            z_min=self.z_min,
            cluster_dist=self.cluster_dist,
            min_passage_width=self.min_passage_width,
            logger=self.get_logger(),
        )

        self.coordinator = Coordinator(
            robot_id=self.robot_id,
            robot_count=self.robot_count,
            dispersion_threshold=self.dispersion_threshold,
            exclusion_radius=self.coordination_exclusion_radius,
            reservation_ttl=self.reservation_ttl,
            logger=self.get_logger(),
        )

        self._latest_cloud: Optional[PointCloud2] = None
        self._latest_grid: Optional[np.ndarray] = None
        self._robot_has_frontier: Dict[int, bool] = {}

        qos_transient = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        qos_volatile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )

        self.create_subscription(
            PointCloud2, f'/{self.namespace}/pointcloud_real',
            self._cloud_cb, 10
        )

        self.create_subscription(
            PoseGraph, '/cslam/viz/pose_graph',
            self._pose_graph_cb, 10
        )

        self.create_subscription(
            PoseStamped, '/frontier/reservations',
            self._reservations_cb, qos_transient
        )

        self.create_subscription(
            Path, f'/{self.namespace}/darp/route',
            self._darp_route_cb, qos_transient
        )

        self._grid_pub = self.create_publisher(
            OccupancyGrid, '/frontier/grid', qos_transient
        )
        self.create_subscription(
            OccupancyGrid, '/frontier/grid', self._grid_cb, qos_volatile
        )

        self._goal_pub = self.create_publisher(
            PoseStamped, f'/{self.namespace}/frontier/goal', 10
        )
        self._reservation_pub = self.create_publisher(
            PoseStamped, '/frontier/reservations', qos_transient
        )
        self._wake_up_pub = self.create_publisher(
            WakeUp, '/darp/wake_up', 10
        )

        self._frontier_status_pub = self.create_publisher(
            UInt8MultiArray, '/frontier/frontier_status', 10
        )
        self.create_subscription(
            UInt8MultiArray, '/frontier/frontier_status',
            self._frontier_status_cb, 10
        )

        self.create_subscription(
            AnomalyDetected, '/anomaly_detection/anomaly',
            self._anomaly_callback, 10
        )

        self._anomaly_cleared_pub = self.create_publisher(
            Int32, '/frontier/anomaly_cleared', 10
        )
        self.create_subscription(
            Int32, '/frontier/anomaly_cleared',
            self._anomaly_cleared_cb, 10
        )

        self.get_logger().info(
            f'Exploration node started for robot {self.robot_id}. '
            f'Namespace={self.namespace}, frame={self.global_frame}'
        )

    def _wait_for_lifecycle_nodes(self) -> None:
        nodes = ['controller_server', 'planner_server', 'behavior_server', 'smoother_server', 'velocity_smoother']
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
        self.get_logger().info('All lifecycle nodes active. Waiting 3s for stabilization...')
        time.sleep(3)

    def _get_current_pose(self) -> Optional[Tuple[float, float]]:
        try:
            latest = rclpy.time.Time(seconds=0)
            transform = self.tf_buffer.lookup_transform(
                self.nav_frame, self.base_frame, latest
            )
            return (transform.transform.translation.x, transform.transform.translation.y)
        except Exception:
            return None

    def _get_pose_in_frame(self, target_frame: str) -> Optional[Tuple[float, float]]:
        try:
            latest = rclpy.time.Time(seconds=0)
            transform = self.tf_buffer.lookup_transform(
                target_frame, self.base_frame, latest
            )
            return (transform.transform.translation.x, transform.transform.translation.y)
        except Exception:
            return None

    def _tf_cb(self, msg: TFMessage) -> None:
        for t in msg.transforms:
            try:
                self.tf_buffer.set_transform(t, 'default_authority')
            except Exception:
                pass

    def _tf_static_cb(self, msg: TFMessage) -> None:
        for t in msg.transforms:
            try:
                self.tf_buffer.set_transform_static(t, 'default_authority')
            except Exception:
                pass

    def _cloud_cb(self, msg: PointCloud2) -> None:
        self._latest_cloud = msg

    def _pose_graph_cb(self, msg: PoseGraph) -> None:
        poses: Dict[int, Tuple[float, float]] = {}
        values = list(getattr(msg, "values", []))
        for v in values:
            rid = int(v.key.robot_id)
            pose = v.pose
            poses[rid] = (float(pose.position.x), float(pose.position.y))
        self.coordinator.update_robot_poses(poses)

    def _reservations_cb(self, msg: PoseStamped) -> None:
        frame = msg.header.frame_id
        if not frame:
            return
        try:
            parts = frame.split('_')
            if len(parts) >= 2 and parts[0] == 'robot':
                rid = int(parts[1])
            else:
                return
        except (ValueError, IndexError):
            return

        if rid == self.robot_id:
            return

        if msg.pose.position.z < 0:
            self.coordinator.clear_other_reservation(rid)
            return

        try:
            transformed = self.tf_buffer.transform(msg, self.global_frame)
            self.coordinator.update_reservation(
                rid, transformed.pose.position.x, transformed.pose.position.y
            )
        except Exception as e:
            self.get_logger().warn(f'Cannot transform reservation from {frame}: {e}')

    def _darp_route_cb(self, msg: Path) -> None:
        if len(msg.poses) > 0:
            self.get_logger().info('DARP route received, exploration continues.')

    def _frontier_status_cb(self, msg: UInt8MultiArray) -> None:
        if len(msg.data) < 2:
            return
        rid = msg.data[0]
        self._robot_has_frontier[rid] = bool(msg.data[1])

    def _publish_frontier_status(self, has_frontier: bool) -> None:
        self._robot_has_frontier[self.robot_id] = has_frontier
        msg = UInt8MultiArray()
        msg.data = [self.robot_id, 1 if has_frontier else 0]
        self._frontier_status_pub.publish(msg)

    def _all_robots_done(self) -> bool:
        if len(self._robot_has_frontier) < self.robot_count:
            return False
        return not any(self._robot_has_frontier.values())

    def _start_exploration_timer(self) -> None:
        if self._exploration_timer is not None:
            self._exploration_timer.cancel()
        self._exploration_timer = self.create_timer(
            self.exploration_period, self._exploration_loop
        )

    def _exploration_loop(self) -> None:
        self.get_logger().info(f'[_exploration_loop] state={self._state}')
        try:
            self._exploration_loop_body()
        except Exception as e:
            self.get_logger().error(f'[_exploration_loop] UNHANDLED EXCEPTION: {e}', throttle_duration_sec=5.0)

    def _exploration_loop_body(self) -> None:
        if self._state == ExplorationState.EXPLORING:
            return

        if self._state == ExplorationState.DONE:
            return

        if self._latest_cloud is None:
            self.get_logger().info(f'[_exploration_loop] no cloud yet, waiting', throttle_duration_sec=5.0)
            return

        from rclpy.time import Time

        if self.tf_buffer.can_transform(self.global_frame, self.nav_frame, Time(seconds=0), timeout=Duration(seconds=0.1)):
            frame_for_frontiers = self.global_frame
            robot_pos_shared = self._get_pose_in_frame(self.global_frame)
        else:
            frame_for_frontiers = self.nav_frame
            robot_pos_shared = self._get_current_pose()
            self.get_logger().info(f'No TF {self.nav_frame} -> {self.global_frame}, using {self.nav_frame}')

        if robot_pos_shared is None:
            self.get_logger().warn('Cannot get robot pose, skipping exploration cycle.')
            return

        my_frontiers, grid = self.frontier_finder.find_frontiers(
            self._latest_cloud, robot_pos_shared[0], robot_pos_shared[1],
            self.tf_buffer, frame_for_frontiers, self.base_frame,
        )
        self._latest_grid = grid
        self.get_logger().info(f'Publishing grid in {frame_for_frontiers}')
        self._publish_grid(frame_for_frontiers)

        if not my_frontiers:
            self._publish_frontier_status(False)
            if self._all_robots_done():
                self.get_logger().info('All robots have no frontiers — exploration complete')
                self._call_darp_wake_up()
                self._state = ExplorationState.DONE
            else:
                self.get_logger().info('No frontiers locally, waiting for other robots')
                self._start_exploration_timer()
            return

        frontier_tuples = [(f.x, f.y, f.dist_to_robot, f.unknown_ratio) for f in my_frontiers]
        candidates = self.coordinator.select_best_frontiers(frontier_tuples, robot_pos_shared)
        self.get_logger().info(f'select_best_frontiers returned {len(candidates)} candidates')

        if not candidates:
            self._publish_frontier_status(False)
            if self._all_robots_done():
                self.get_logger().info('All robots have no frontiers — exploration complete')
                self._call_darp_wake_up()
                self._state = ExplorationState.DONE
            else:
                self.get_logger().info('Frontiers blocked by coordination, waiting for next cycle')
                self._start_exploration_timer()
            return

        best = self._find_reachable_goal(candidates)
        if best is None:
            self.get_logger().warn('No reachable frontier goal found')
            self._publish_frontier_status(False)
            self._start_exploration_timer()
            return

        self._publish_frontier_status(True)
        self._publish_reservation(best[0], best[1])
        self._send_goal(best[0], best[1])
        self._state = ExplorationState.EXPLORING
        self._current_goal = best
        self._goal_start_time = time.time()

        if self._monitor_timer is not None:
            self._monitor_timer.cancel()
        self._monitor_timer = self.create_timer(1.0, self._monitor_loop)

    def _publish_grid(self, frame: str = '') -> None:
        grid = self.frontier_finder._persistent_grid
        unknown = int(np.sum(grid == -1))
        free = int(np.sum(grid == 0))
        occupied = int(np.sum(grid == 100))
        self.get_logger().info(
            f'Publishing grid in {frame} — unknown={unknown} free={free} occupied={occupied}'
        )
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame if frame else self.global_frame
        msg.info.resolution = 0.1
        msg.info.width = grid.shape[1]
        msg.info.height = grid.shape[0]
        msg.info.origin.position.x = -20.0
        msg.info.origin.position.y = -20.0
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0
        msg.data = grid.flatten().astype(np.int8).tolist()
        self._grid_pub.publish(msg)

    def _grid_cb(self, msg: OccupancyGrid) -> None:
        if msg.header.frame_id != self.global_frame:
            self.get_logger().warn(f'Ignored grid in {msg.header.frame_id}, expected {self.global_frame}')
            return
        if abs(msg.info.resolution - 0.1) > 1e-6 or msg.info.width != 400 or msg.info.height != 400:
            return
        received = np.array(msg.data, dtype=np.int8).reshape(400, 400)
        r_unknown = int(np.sum(received == -1))
        r_free = int(np.sum(received == 0))
        r_occupied = int(np.sum(received == 100))
        local = self.frontier_finder._persistent_grid
        l_unknown = int(np.sum(local == -1))
        l_free = int(np.sum(local == 0))
        l_occupied = int(np.sum(local == 100))
        np.maximum(local, received, out=local)
        if self._anomaly_mask is not None:
            revert = self._anomaly_mask & (local != -1)
            n_revert = int(np.sum(revert))
            if n_revert > 0:
                local[revert] = -1
                self.get_logger().info(
                    f'Anomaly mask protected {n_revert} cells from being overwritten'
                )

        a_unknown = int(np.sum(local == -1))
        a_free = int(np.sum(local == 0))
        a_occupied = int(np.sum(local == 100))
        self.get_logger().info(
            f'Grid merge — received: u={r_unknown} f={r_free} o={r_occupied} | '
            f'local before: u={l_unknown} f={l_free} o={l_occupied} | '
            f'after: u={a_unknown} f={a_free} o={a_occupied} | '
            f'Δunknown={l_unknown - a_unknown} Δfree={a_free - l_free} Δoccupied={a_occupied - l_occupied}'
        )

    def _find_reachable_goal(
        self, candidates: List[Tuple[float, float, float]]
    ) -> Optional[Tuple[float, float]]:
        robot_pos = self._get_current_pose()
        if robot_pos is None:
            self.get_logger().warn('Cannot get robot pose for reachability check')
            return None

        start = PoseStamped()
        start.header.frame_id = self.nav_frame
        start.pose.position.x = robot_pos[0]
        start.pose.position.y = robot_pos[1]
        start.pose.position.z = 0.0
        start.pose.orientation.w = 1.0

        for x, y, score in candidates:
            goal = PoseStamped()
            goal.header.frame_id = self.nav_frame
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.pose.position.x = x
            goal.pose.position.y = y
            goal.pose.position.z = 0.0
            goal.pose.orientation.w = 1.0

            try:
                path = self.navigator.getPath(start, goal, planner_id='', use_start=True)
                if path is not None and len(path.poses) > 0:
                    self.get_logger().info(f'Reachable goal: ({x:.2f}, {y:.2f}), score={score:.2f}')
                    return (x, y)
                self.get_logger().warn(f'Goal ({x:.2f}, {y:.2f}) unreachable (empty path), trying next')
            except Exception as e:
                self.get_logger().warn(f'Path check failed for ({x:.2f}, {y:.2f}): {e}')

        self.get_logger().warn('All candidates unreachable, waiting for next cycle')
        return None

    def _publish_reservation(self, x: float, y: float) -> None:
        self.coordinator.set_my_reservation(x, y)

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.nav_frame
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.orientation.w = 1.0

        self._reservation_pub.publish(msg)

    def _clear_reservation(self) -> None:
        self.coordinator.clear_my_reservation()

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.nav_frame
        msg.pose.position.x = 0.0
        msg.pose.position.y = 0.0
        msg.pose.position.z = -1.0
        msg.pose.orientation.w = 1.0

        self._reservation_pub.publish(msg)

    def _send_goal(self, x: float, y: float) -> None:
        import math
        robot_pos = self._get_current_pose()

        pose = PoseStamped()
        pose.header.frame_id = self.nav_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0

        if robot_pos is not None:
            dx = x - robot_pos[0]
            dy = y - robot_pos[1]
            dist = math.hypot(dx, dy)
            yaw = math.atan2(dy, dx)
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            self.get_logger().info(f'Sending exploration goal: ({x:.2f}, {y:.2f}), dist={dist:.2f}m')
        else:
            pose.pose.orientation.w = 1.0

        self._goal_pub.publish(pose)
        self.navigator.cancelTask()
        self.navigator.goToPose(pose)

    def _monitor_loop(self) -> None:
        if self._state != ExplorationState.EXPLORING:
            return

        if self._current_goal is None:
            return

        if self.navigator.isTaskComplete():
            result = self.navigator.getResult()
            self._clear_reservation()

            if result == TaskResult.SUCCEEDED:
                self.get_logger().info('Exploration goal reached.')
            elif result == TaskResult.FAILED:
                self.get_logger().warn('Exploration goal failed.')
            elif result == TaskResult.CANCELED:
                self.get_logger().info('Exploration goal canceled.')
            else:
                self.get_logger().info(f'Exploration goal result: {result}')

            self._state = ExplorationState.IDLE
            self._current_goal = None
            self._goal_start_time = None
            self.get_logger().info('[monitor] Goal done, state=IDLE, restarting exploration timer')

            if self._monitor_timer is not None:
                self._monitor_timer.cancel()
                self._monitor_timer = None

            self._start_exploration_timer()

        else:
            if self._goal_start_time is not None:
                elapsed = time.time() - self._goal_start_time
                if elapsed > self.frontier_timeout:
                    self.get_logger().warn(
                        f'Exploration goal timeout ({elapsed:.1f}s > {self.frontier_timeout}s), aborting.'
                    )
                    self._clear_reservation()
                    self._state = ExplorationState.IDLE
                    self._current_goal = None
                    self._goal_start_time = None

                    if self._monitor_timer is not None:
                        self._monitor_timer.cancel()
                        self._monitor_timer = None

                    self._start_exploration_timer()

    def _call_darp_wake_up(self) -> None:
        self.get_logger().info('Exploration finished. Waking up DARP.')
        if self._anomaly_mask is not None and not self._anomaly_cleared_sent:
            cleared_msg = Int32()
            cleared_msg.data = self.robot_id
            self._anomaly_cleared_pub.publish(cleared_msg)
            self._anomaly_cleared_sent = True
            self.get_logger().info('Published anomaly_cleared — all robots may drop anomaly mask')

        msg = WakeUp()
        msg.resolution = 0.5
        msg.padding = 0.0
        msg.obstacle_dilation = 1
        msg.use_equal_portions = True
        msg.portions = []
        msg.active_robot_ids = list(range(self.robot_count))
        self._wake_up_pub.publish(msg)

    def _anomaly_callback(self, msg: AnomalyDetected) -> None:
        self.get_logger().warn("Anomaly POG")
        ax = msg.pose.position.x
        ay = msg.pose.position.y
        if self.nav_frame != self.global_frame:
            try:
                pt = PointStamped()
                pt.header.frame_id = self.nav_frame
                pt.header.stamp = self.get_clock().now().to_msg()
                pt.point.x = ax
                pt.point.y = ay
                transformed = self.tf_buffer.transform(pt, self.global_frame)
                ax = transformed.point.x
                ay = transformed.point.y
            except Exception:
                self.get_logger().warn(
                    f'Could not transform anomaly from {self.nav_frame} to {self.global_frame}, '
                    f'using raw coordinates'
                )

        r, c = self.frontier_finder._world_to_grid(ax, ay)
        radius_cells = 10
        r0 = max(0, r - radius_cells)
        r1 = min(self.frontier_finder._rows, r + radius_cells + 1)
        c0 = max(0, c - radius_cells)
        c1 = min(self.frontier_finder._cols, c + radius_cells + 1)

        old = self.frontier_finder._persistent_grid[r0:r1, c0:c1].copy()
        if np.all(old == -1):
            self.get_logger().info(
                f'Anomaly at ({ax:.2f}, {ay:.2f}) grid=({r},{c}) — area already unknown, skipping'
            )
            return

        new_mask = np.zeros_like(self.frontier_finder._persistent_grid, dtype=bool)
        new_mask[r0:r1, c0:c1] = True
        if self._anomaly_mask is None:
            self._anomaly_mask = new_mask
        else:
            self._anomaly_mask |= new_mask

        self.frontier_finder._persistent_grid[r0:r1, c0:c1] = -1
        self._anomaly_cleared_sent = False

        self.get_logger().warn(
            f'Anomaly from robot{msg.robot_id} at ({ax:.2f}, {ay:.2f}) grid=({r},{c}), '
            f'cleared {r1 - r0}x{c1 - c0} cells to unknown'
        )

        if msg.robot_id == self.robot_id:
            self._on_anomaly()

    def _on_anomaly(self) -> None:
        self._clear_reservation()
        self._state = ExplorationState.IDLE
        self._current_goal = None

        if self._monitor_timer is not None:
            self._monitor_timer.cancel()
            self._monitor_timer = None

        self._start_exploration_timer()
        self._exploration_loop()

    def _anomaly_cleared_cb(self, msg: Int32) -> None:
        self.get_logger().info(
            f'Received anomaly_cleared from robot{msg.data}, dropping anomaly mask'
        )
        self._anomaly_mask = None
        self._anomaly_cleared_sent = False

    def start(self) -> None:
        self._start_exploration_timer()


def main(args=None) -> None:
    signal.signal(signal.SIGINT, signal.default_int_handler)
    rclpy.init(args=args)
    node = ExplorationNode()

    try:
        node.start()
        rclpy.spin(node)
    finally:
        node.get_logger().info('Shutting down, cancelling navigation goal.')
        node.navigator.cancelTask()
        node._clear_reservation()
        if node._exploration_timer:
            node._exploration_timer.cancel()
        if node._monitor_timer:
            node._monitor_timer.cancel()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()