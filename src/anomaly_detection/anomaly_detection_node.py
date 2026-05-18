#!/usr/bin/env python3
from __future__ import annotations

import math
import threading
from collections import defaultdict
from copy import deepcopy
from dataclasses import dataclass
from typing import Dict, List, Optional, Set, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener
from tf2_msgs.msg import TFMessage

from cslam_common_interfaces.msg import PoseGraph
from anomaly_detection.msg import AnomalyDetected

TF_TIMEOUT = Duration(seconds=2.0)

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


def transform_matrix_from_tf(buffer: Buffer, target_frame: str, source_frame: str) -> Optional[np.ndarray]:
    if not target_frame or not source_frame or target_frame == source_frame:
        return np.eye(4, dtype=np.float64)
    try:
        tf = buffer.lookup_transform(target_frame, source_frame, Time(), timeout=TF_TIMEOUT)
    except Exception:
        return None

    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = quat_to_rot_matrix(tf.transform.rotation)
    T[0, 3] = float(tf.transform.translation.x)
    T[1, 3] = float(tf.transform.translation.y)
    T[2, 3] = float(tf.transform.translation.z)
    return T


def transform_point(T: np.ndarray, xyz: Tuple[float, float, float]) -> Tuple[float, float, float]:
    v = np.array([xyz[0], xyz[1], xyz[2], 1.0], dtype=np.float64)
    w = T @ v
    return float(w[0]), float(w[1]), float(w[2])


def bresenham_cells(r0: int, c0: int, r1: int, c1: int) -> List[Tuple[int, int]]:
    cells: List[Tuple[int, int]] = []
    dr = abs(r1 - r0)
    dc = abs(c1 - c0)
    sr = 1 if r0 < r1 else -1
    sc = 1 if c0 < c1 else -1
    err = dc - dr
    r, c = r0, c0
    while True:
        cells.append((r, c))
        if r == r1 and c == c1:
            break
        e2 = 2 * err
        if e2 > -dr:
            err -= dr
            c += sc
        if e2 < dc:
            err += dc
            r += sr
    return cells


@dataclass
class GridInfo:
    msg: OccupancyGrid
    stamp_key: Tuple[int, int]


@dataclass
class AnomalyEvent:
    robot_id: int
    anomaly_id: int
    anomaly_type: str
    cell_r: int
    cell_c: int
    anomaly_x: float
    anomaly_y: float
    frame_id: str
    stamp_sec: int
    stamp_nanosec: int
    evidence_points: int


class AnomalyDetectionNode(Node):
    def __init__(self) -> None:
        super().__init__("anomaly_detection_node")
        self.cb_group = ReentrantCallbackGroup()

        self.declare_parameter("robot_prefix", "r")
        self.declare_parameter("robot_id", 0)
        self.declare_parameter("grid_topic", "darp/full_area")
        self.declare_parameter("anomaly_topic", "/anomaly_detection/anomaly")
        self.declare_parameter("marker_topic", "/anomaly_detection/marker")
        self.declare_parameter("pose_topic", "/cslam/viz/pose_graph")
        self.declare_parameter("pointcloud_topic", "pointcloud_real")
        self.declare_parameter("base_frame", "laser_frame")
        self.declare_parameter("occupied_threshold", 50)
        self.declare_parameter("z_min", 0.30)
        self.declare_parameter("z_max", 2.00)
        self.declare_parameter("min_points_per_cell", 10)
        self.declare_parameter("stable_frames", 10)
        self.declare_parameter("cooldown_sec", 1.0)
        self.declare_parameter("max_neighbors_for_disappearance", 0)

        self.robot_prefix = str(self.get_parameter("robot_prefix").value)
        self.robot_id = int(self.get_parameter("robot_id").value)
        self.grid_topic = str(self.get_parameter("grid_topic").value)
        self.anomaly_topic = str(self.get_parameter("anomaly_topic").value)
        self.marker_topic = str(self.get_parameter("marker_topic").value)
        self.pose_topic = str(self.get_parameter("pose_topic").value)
        self.pointcloud_topic = str(self.get_parameter("pointcloud_topic").value)
        self.input_base_frame = str(self.get_parameter("base_frame").value)
        self.occupied_threshold = int(self.get_parameter("occupied_threshold").value)
        self.z_min = float(self.get_parameter("z_min").value)
        self.z_max = float(self.get_parameter("z_max").value)
        self.min_points_per_cell = int(self.get_parameter("min_points_per_cell").value)
        self.stable_frames = int(self.get_parameter("stable_frames").value)
        self.cooldown_sec = float(self.get_parameter("cooldown_sec").value)
        self.max_neighbors_for_disappearance = int(self.get_parameter("max_neighbors_for_disappearance").value)

        self.namespace = f"{self.robot_prefix}{self.robot_id}"
        self.base_frame = f"{self.namespace}/{self.input_base_frame}"
        self.global_frame = f"robot{self.robot_id}_map"
        self.full_area_topic = f"/{self.namespace}/{self.grid_topic}"
        self.cloud_topic = f"/{self.namespace}/{self.pointcloud_topic}"
        self.tf_topic = "/tf"
        self.tf_static_topic = f"/{self.namespace}/tf_static"

        self.tf_buffer = Buffer()

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

        self.create_subscription(TFMessage, self.tf_topic, self._tf_cb, tf_qos, callback_group=self.cb_group)
        self.create_subscription(TFMessage, self.tf_static_topic, self._tf_static_cb, static_qos, callback_group=self.cb_group)

        self.lock = threading.Lock()
        self.latest_grid: Optional[GridInfo] = None
        self.latest_pose: Optional[Pose] = None
        self.latest_pose_frame: str = ""

        self.appearance_streak: Dict[Tuple[int, int], int] = {}
        self.disappearance_streak: Dict[Tuple[int, int], int] = {}
        self.last_published: Dict[Tuple[str, int, int], float] = {}
        self.next_anomaly_id = 1
        self.published_ids: Set[Tuple[int, int]] = set()
        self.max_published_ids = 10000

        self.anomaly_pub = self.create_publisher(AnomalyDetected, self.anomaly_topic, 10)
        self.marker_pub = self.create_publisher(MarkerArray, self.marker_topic, 10)

        self.grid_sub = self.create_subscription(
            OccupancyGrid, self.full_area_topic, self._grid_cb, 10,
            callback_group=self.cb_group)
        self.cloud_sub = self.create_subscription(
            PointCloud2, self.cloud_topic, self._cloud_cb, 10,
            callback_group=self.cb_group)
        self.pose_sub = self.create_subscription(
            PoseGraph, self.pose_topic, self._pose_graph_cb, 10,
            callback_group=self.cb_group)

        self.get_logger().info(
            f"Started. grid: {self.full_area_topic}, cloud: {self.cloud_topic}, "
            f"pose: {self.pose_topic}, markers: {self.marker_topic}")

        self.get_logger().info(
            f"Waiting for occupancy grid {self.full_area_topic} to be published...")

    def _tf_cb(self, msg: TFMessage) -> None:
        for transform in msg.transforms:
            self.tf_buffer.set_transform(transform, 'default_authority')

    def _tf_static_cb(self, msg: TFMessage) -> None:
        for transform in msg.transforms:
            self.tf_buffer.set_transform_static(transform, 'default_authority')

    def _grid_cb(self, msg: OccupancyGrid) -> None:
        stamp_key = (int(msg.header.stamp.sec), int(msg.header.stamp.nanosec))
        with self.lock:
            prev = self.latest_grid
            prev_key = prev.stamp_key if prev is not None else None
            self.latest_grid = GridInfo(msg=deepcopy(msg), stamp_key=stamp_key)
            if prev_key != stamp_key:
                self.get_logger().info(
                    f"Got new occupancy grid, seeking for anomalies...")
                self.appearance_streak.clear()
                self.disappearance_streak.clear()

    def _extract_latest_pose(self, msg: PoseGraph) -> Optional[Pose]:
        values = list(getattr(msg, "values", []))
        for v in values:
            if int(v.key.robot_id) == self.robot_id:
                return deepcopy(v.pose)
        return None

    def _pose_graph_cb(self, msg: PoseGraph) -> None:
        pose = self._extract_latest_pose(msg)
        if pose is not None:
            with self.lock:
                self.latest_pose = pose
                self.latest_pose_frame = self.global_frame

    def _robot_pose_in_grid_frame(self, grid_frame: str) -> Optional[PoseStamped]:
        T_grid_from_base = transform_matrix_from_tf(self.tf_buffer, grid_frame, self.base_frame)
        if T_grid_from_base is not None:
            pose = PoseStamped()
            pose.header.frame_id = grid_frame
            pose.header.stamp = self.get_clock().now().to_msg()
            x, y, z = transform_point(T_grid_from_base, (0.0, 0.0, 0.0))
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = z
            pose.pose.orientation.w = 1.0
            return pose

        with self.lock:
            pose_msg = self.latest_pose
            pose_frame = self.latest_pose_frame

        if pose_msg is None:
            return None

        if pose_frame and pose_frame != grid_frame:
            T_grid_from_pose = transform_matrix_from_tf(self.tf_buffer, grid_frame, pose_frame)
            if T_grid_from_pose is not None:
                p = PoseStamped()
                p.header.frame_id = grid_frame
                p.header.stamp = self.get_clock().now().to_msg()
                x, y, z = transform_point(
                    T_grid_from_pose,
                    (float(pose_msg.position.x), float(pose_msg.position.y), float(pose_msg.position.z)),
                )
                p.pose.position.x = x
                p.pose.position.y = y
                p.pose.position.z = z
                p.pose.orientation = deepcopy(pose_msg.orientation)
                return p

        p = PoseStamped()
        p.pose = deepcopy(pose_msg)
        p.header.frame_id = grid_frame
        p.header.stamp = self.get_clock().now().to_msg()
        return p

    def _cloud_cb(self, msg: PointCloud2) -> None:
        with self.lock:
            grid_info = self.latest_grid

        if grid_info is None:
            self.get_logger().debug("No occupancy grid yet.")
            return

        grid = grid_info.msg
        grid_frame = grid.header.frame_id

        robot_pose_in_grid = self._robot_pose_in_grid_frame(grid_frame)
        if robot_pose_in_grid is None:
            self.get_logger().warn(
                f"Cannot determine robot pose in frame '{grid_frame}', skipping scan.")
            return

        T_grid_from_cloud = transform_matrix_from_tf(self.tf_buffer, grid_frame, self.base_frame)
        if T_grid_from_cloud is None:
            self.get_logger().warn(
                f"TF missing for {self.base_frame} -> {grid_frame}, skipping scan.")
            return

        self.get_logger().debug(
            f"Robot pose in grid: {robot_pose_in_grid.pose.position.x}, {robot_pose_in_grid.pose.position.y}")

        robot_rc = self._world_to_cell(grid, robot_pose_in_grid.pose.position.x, robot_pose_in_grid.pose.position.y)
        if robot_rc is None:
            self.get_logger().warn("Robot pose outside grid bounds, skipping scan.")
            return

        appearance_hits: Dict[Tuple[int, int], int] = defaultdict(int)
        disappearance_hits: Dict[Tuple[int, int], int] = defaultdict(int)
        all_occupied_cells: Set[Tuple[int, int]] = set()

        for pt in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            if not (math.isfinite(pt[0]) and math.isfinite(pt[1]) and math.isfinite(pt[2])):
                continue
            x_g, y_g, z_g = transform_point(T_grid_from_cloud, (float(pt[0]), float(pt[1]), float(pt[2])))
            if z_g < self.z_min or z_g > self.z_max:
                continue

            hit_rc = self._world_to_cell(grid, x_g, y_g)
            if hit_rc is None:
                continue
            hr, hc = hit_rc
            all_occupied_cells.add((hr, hc))

            cell_value = self._cell_value(grid, hr, hc)
            if cell_value is not None and 0 <= cell_value < self.occupied_threshold:
                appearance_hits[(hr, hc)] += 1

            ray_cells = bresenham_cells(robot_rc[0], robot_rc[1], hr, hc)
            if len(ray_cells) >= 2:
                for rr, cc in ray_cells[:-1]:
                    v = self._cell_value(grid, rr, cc)
                    if v is not None and v > self.occupied_threshold:
                        disappearance_hits[(rr, cc)] += 1

        self._update_and_publish(grid, robot_pose_in_grid, appearance_hits, disappearance_hits, all_occupied_cells)

    def _update_and_publish(
        self,
        grid: OccupancyGrid,
        robot_pose: PoseStamped,
        appearance_hits: Dict[Tuple[int, int], int],
        disappearance_hits: Dict[Tuple[int, int], int],
        all_occupied_cells: Set[Tuple[int, int]],
    ) -> None:
        now_sec = float(self.get_clock().now().nanoseconds) / 1e9

        self._update_streaks_and_publish_type(
            grid=grid,
            robot_pose=robot_pose,
            hits=appearance_hits,
            streaks=self.appearance_streak,
            anomaly_type="appearance",
            now_sec=now_sec,
            all_occupied_cells=all_occupied_cells,
        )
        self._update_streaks_and_publish_type(
            grid=grid,
            robot_pose=robot_pose,
            hits=disappearance_hits,
            streaks=self.disappearance_streak,
            anomaly_type="disappearance",
            now_sec=now_sec,
            all_occupied_cells=all_occupied_cells,
        )

    def _update_streaks_and_publish_type(
        self,
        grid: OccupancyGrid,
        robot_pose: PoseStamped,
        hits: Dict[Tuple[int, int], int],
        streaks: Dict[Tuple[int, int], int],
        anomaly_type: str,
        now_sec: float,
        all_occupied_cells: Set[Tuple[int, int]],
    ) -> None:
        stale_keys = set(streaks.keys()) - set(hits.keys())
        for key in stale_keys:
            streaks[key] = 0

        for cell, evidence_count in hits.items():
            if evidence_count < self.min_points_per_cell:
                streaks[cell] = 0
                continue

            if anomaly_type == "disappearance":
                num_neighbors = self._count_occupied_neighbors_in_set(all_occupied_cells, cell[0], cell[1])
                if num_neighbors > self.max_neighbors_for_disappearance:
                    streaks[cell] = 0
                    continue

            streaks[cell] = int(streaks.get(cell, 0)) + 1
            if streaks[cell] < self.stable_frames:
                continue

            cooldown_key = (anomaly_type, cell[0], cell[1])
            last_t = self.last_published.get(cooldown_key, -1e9)
            if now_sec - last_t < self.cooldown_sec:
                continue

            event_x = float(grid.info.origin.position.x + (cell[1] + 0.5) * grid.info.resolution)
            event_y = float(grid.info.origin.position.y + (cell[0] + 0.5) * grid.info.resolution)

            event = AnomalyEvent(
                robot_id=self.robot_id,
                anomaly_id=self.next_anomaly_id,
                anomaly_type=anomaly_type,
                cell_r=cell[0],
                cell_c=cell[1],
                anomaly_x=event_x,
                anomaly_y=event_y,
                frame_id=grid.header.frame_id or "map",
                stamp_sec=int(grid.header.stamp.sec),
                stamp_nanosec=int(grid.header.stamp.nanosec),
                evidence_points=int(evidence_count),
            )
            self.next_anomaly_id += 1
            self.last_published[cooldown_key] = now_sec
            self._publish_event(event)

    def _publish_event(self, event: AnomalyEvent) -> None:
        msg = AnomalyDetected()
        msg.id = int(event.anomaly_id)
        msg.timestamp.sec = int(event.stamp_sec)
        msg.timestamp.nanosec = int(event.stamp_nanosec)
        msg.pose.position.x = float(event.anomaly_x)
        msg.pose.position.y = float(event.anomaly_y)
        msg.pose.position.z = 0.0
        msg.pose.orientation.w = 1.0
        msg.type = str(event.anomaly_type)
        msg.robot_id = int(event.robot_id)

        self.anomaly_pub.publish(msg)
        if (event.anomaly_x, event.anomaly_y) not in self.published_ids:
            self.published_ids.add((event.anomaly_x, event.anomaly_y))
            if len(self.published_ids) > self.max_published_ids:
                self.published_ids.clear()
            self.get_logger().warn(
                f"Anomaly detected: robot={event.robot_id} type={event.anomaly_type} "
                f"cell=({event.cell_r},{event.cell_c}) id={event.anomaly_id}")

        is_appearance = event.anomaly_type == "appearance"
        sphere = Marker()
        sphere.header.frame_id = event.frame_id
        sphere.header.stamp = self.get_clock().now().to_msg()
        sphere.ns = "anomaly"
        sphere.id = int(event.anomaly_id)
        sphere.type = Marker.SPHERE
        sphere.action = Marker.ADD
        sphere.pose.position.x = float(event.anomaly_x)
        sphere.pose.position.y = float(event.anomaly_y)
        sphere.pose.position.z = 0.2
        sphere.pose.orientation.w = 1.0
        sphere.scale.x = 0.3
        sphere.scale.y = 0.3
        sphere.scale.z = 0.3
        if is_appearance:
            sphere.color.r = 1.0
            sphere.color.g = 0.0
            sphere.color.b = 0.0
        else:
            sphere.color.r = 0.0
            sphere.color.g = 0.0
            sphere.color.b = 1.0
        sphere.color.a = 0.8
        sphere.lifetime = rclpy.duration.Duration(seconds=2.0).to_msg()

        markers = MarkerArray()
        markers.markers.append(sphere)
        self.marker_pub.publish(markers)

    def _cell_value(self, grid: OccupancyGrid, r: int, c: int) -> Optional[int]:
        if r < 0 or c < 0 or r >= grid.info.height or c >= grid.info.width:
            return None
        idx = r * grid.info.width + c
        if idx < 0 or idx >= len(grid.data):
            return None
        return int(grid.data[idx])

    def _count_occupied_neighbors_in_set(self, occupied_set: Set[Tuple[int, int]], r: int, c: int) -> int:
        count = 0
        for dr in [-1, 0, 1]:
            for dc in [-1, 0, 1]:
                if dr == 0 and dc == 0:
                    continue
                if (r + dr, c + dc) in occupied_set:
                    count += 1
        return count

    def _world_to_cell(self, grid: OccupancyGrid, x: float, y: float) -> Optional[Tuple[int, int]]:
        res = float(grid.info.resolution)
        ox = float(grid.info.origin.position.x)
        oy = float(grid.info.origin.position.y)
        c = int(math.floor((x - ox) / res))
        r = int(math.floor((y - oy) / res))
        if 0 <= r < grid.info.height and 0 <= c < grid.info.width:
            return r, c
        return None


def main(args=None) -> None:
    import signal
    signal.signal(signal.SIGINT, signal.default_int_handler)
    rclpy.init(args=args)
    node = AnomalyDetectionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
