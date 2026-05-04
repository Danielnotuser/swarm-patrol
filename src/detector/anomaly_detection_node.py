#!/usr/bin/env python3
from __future__ import annotations

import math
import threading
from collections import defaultdict
from copy import deepcopy
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import OccupancyGrid
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from anomaly_detection.msg import AnomalyDetected
from tf2_ros import Buffer, TransformListener

try:
    from tf2_geometry_msgs import do_transform_pose_stamped  # type: ignore
except Exception:  # pragma: no cover
    do_transform_pose_stamped = None


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
        tf = buffer.lookup_transform(target_frame, source_frame, Time(seconds=0))
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


def parse_robot_index(text: str) -> Optional[int]:
    import re
    m = re.search(r"robot(\d+)", text or "")
    return int(m.group(1)) if m else None


@dataclass
class GridInfo:
    msg: OccupancyGrid
    stamp_key: Tuple[int, int]


@dataclass
class AnomalyEvent:
    robot_id: int
    anomaly_id: int
    anomaly_type: str  # appearance | disappearance
    cell_r: int
    cell_c: int
    anomaly_x: float
    anomaly_y: float
    frame_id: str
    stamp_sec: int
    stamp_nanosec: int
    evidence_points: int


class AnomalyDetectionNode(Node):
    """
    Compares a robot DARP occupancy grid with live lidar pointclouds.

    The report describes anomaly detection as:
    - comparing the current lidar data with the reference occupancy grid;
    - transforming pointcloud data to the global frame with tf;
    - treating points that fall into free cells as anomaly candidates;
    - signaling only if the anomaly is stable for K frames and exceeds M points;
    - publishing an anomaly message with id, timestamp, pose, type, robot_id.  # report-based fields. fileciteturn6file0
    """

    def __init__(self) -> None:
        super().__init__("anomaly_detection_node")
        self.cb_group = ReentrantCallbackGroup()

        self.declare_parameter("robot_count", 2)
        self.declare_parameter("robot_prefix", "r")
        self.declare_parameter("grid_topic_template", "/{robot}/darp/area")
        self.declare_parameter("pointcloud_topic_template", "/{robot}/pointcloud")
        self.declare_parameter("pose_topic_template", "/{robot}/cslam/current_pose_estimate")
        self.declare_parameter("base_frame_template", "robot{robot}_base_link")
        self.declare_parameter("anomaly_topic", "/anomaly_detected")
        self.declare_parameter("occupied_threshold", 50)
        self.declare_parameter("z_min", 0.10)
        self.declare_parameter("z_max", 2.00)
        self.declare_parameter("min_points_per_cell", 3)
        self.declare_parameter("stable_frames", 3)
        self.declare_parameter("publish_json", True)
        self.declare_parameter("cooldown_sec", 2.0)

        self.robot_count = int(self.get_parameter("robot_count").value)
        self.robot_prefix = str(self.get_parameter("robot_prefix").value)
        self.grid_topic_template = str(self.get_parameter("grid_topic_template").value)
        self.pointcloud_topic_template = str(self.get_parameter("pointcloud_topic_template").value)
        self.pose_topic_template = str(self.get_parameter("pose_topic_template").value)
        self.base_frame_template = str(self.get_parameter("base_frame_template").value)
        self.anomaly_topic = str(self.get_parameter("anomaly_topic").value)
        self.occupied_threshold = int(self.get_parameter("occupied_threshold").value)
        self.z_min = float(self.get_parameter("z_min").value)
        self.z_max = float(self.get_parameter("z_max").value)
        self.min_points_per_cell = int(self.get_parameter("min_points_per_cell").value)
        self.stable_frames = int(self.get_parameter("stable_frames").value)
        self.publish_json = bool(self.get_parameter("publish_json").value)
        self.cooldown_sec = float(self.get_parameter("cooldown_sec").value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.lock = threading.Lock()
        self.latest_grids: Dict[int, GridInfo] = {}
        self.latest_pose: Dict[int, PoseStamped] = {}
        self.latest_pose_frame: Dict[int, str] = {}

        self.appearance_streak: Dict[int, Dict[Tuple[int, int], int]] = defaultdict(dict)
        self.disappearance_streak: Dict[int, Dict[Tuple[int, int], int]] = defaultdict(dict)
        self.last_published: Dict[Tuple[int, str, int, int], float] = {}
        self.next_anomaly_id = 1

        self.anomaly_pub = self.create_publisher(AnomalyDetected, self.anomaly_topic, 10)

        self.grid_subs = []
        self.pointcloud_subs = []
        self.pose_subs = []
        for i in range(self.robot_count):
            robot_ns = f"{self.robot_prefix}{i}"
            grid_topic = self.grid_topic_template.format(robot=robot_ns, id=i)
            cloud_topic = self.pointcloud_topic_template.format(robot=robot_ns, id=i)
            pose_topic = self.pose_topic_template.format(robot=robot_ns, id=i)

            self.grid_subs.append(
                self.create_subscription(
                    OccupancyGrid,
                    grid_topic,
                    lambda msg, rid=i: self._grid_cb(rid, msg),
                    10,
                    callback_group=self.cb_group,
                )
            )
            self.pointcloud_subs.append(
                self.create_subscription(
                    PointCloud2,
                    cloud_topic,
                    lambda msg, rid=i: self._cloud_cb(rid, msg),
                    10,
                    callback_group=self.cb_group,
                )
            )
            self.pose_subs.append(
                self.create_subscription(
                    PoseStamped,
                    pose_topic,
                    lambda msg, rid=i: self._pose_cb(rid, msg),
                    10,
                    callback_group=self.cb_group,
                )
            )

        self.get_logger().info(
            f"Anomaly detector started. Grid topics: {self.grid_topic_template}, clouds: {self.pointcloud_topic_template}"
        )

    # ----------------------- callbacks -----------------------

    def _grid_cb(self, robot_id: int, msg: OccupancyGrid) -> None:
        stamp_key = (int(msg.header.stamp.sec), int(msg.header.stamp.nanosec))
        with self.lock:
            prev = self.latest_grids.get(robot_id)
            prev_key = prev.stamp_key if prev is not None else None
            self.latest_grids[robot_id] = GridInfo(msg=deepcopy(msg), stamp_key=stamp_key)

            # reset history if this robot published a new grid
            if prev_key != stamp_key:
                self.appearance_streak[robot_id].clear()
                self.disappearance_streak[robot_id].clear()

    def _pose_cb(self, robot_id: int, msg: PoseStamped) -> None:
        with self.lock:
            self.latest_pose[robot_id] = deepcopy(msg)
            self.latest_pose_frame[robot_id] = msg.header.frame_id or ""

    def _cloud_cb(self, robot_id: int, msg: PointCloud2) -> None:
        with self.lock:
            grid_info = self.latest_grids.get(robot_id)
            pose_msg = self.latest_pose.get(robot_id)
            pose_frame = self.latest_pose_frame.get(robot_id, "")

        if grid_info is None:
            self.get_logger().debug(f"Robot {robot_id}: no occupancy grid yet.")
            return

        grid = grid_info.msg
        grid_frame = grid.header.frame_id or "map"
        cloud_frame = msg.header.frame_id or ""

        # Robot pose in the grid frame.
        base_frame = self.base_frame_template.format(robot=f"{self.robot_prefix}{robot_id}", id=robot_id)
        robot_pose_in_grid = self._robot_pose_in_grid(robot_id, grid_frame, base_frame, pose_msg, pose_frame)
        if robot_pose_in_grid is None:
            self.get_logger().warn(
                f"Robot {robot_id}: cannot determine robot pose in frame '{grid_frame}', skipping scan."
            )
            return

        T_grid_from_cloud = transform_matrix_from_tf(self.tf_buffer, grid_frame, cloud_frame)
        if T_grid_from_cloud is None:
            self.get_logger().warn(
                f"Robot {robot_id}: TF missing for {cloud_frame} -> {grid_frame}, skipping scan."
            )
            return

        robot_rc = self._world_to_cell(grid, robot_pose_in_grid.pose.position.x, robot_pose_in_grid.pose.position.y)
        if robot_rc is None:
            self.get_logger().warn(f"Robot {robot_id}: robot pose outside grid bounds, skipping scan.")
            return

        appearance_hits: Dict[Tuple[int, int], int] = defaultdict(int)
        disappearance_hits: Dict[Tuple[int, int], int] = defaultdict(int)

        points_in_grid: List[Tuple[float, float, float]] = []
        for pt in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            x_g, y_g, z_g = transform_point(T_grid_from_cloud, (float(pt[0]), float(pt[1]), float(pt[2])))
            if z_g < self.z_min or z_g > self.z_max:
                continue
            points_in_grid.append((x_g, y_g, z_g))

        if not points_in_grid:
            return

        for x_g, y_g, z_g in points_in_grid:
            hit_rc = self._world_to_cell(grid, x_g, y_g)
            if hit_rc is None:
                continue
            hr, hc = hit_rc

            cell_value = self._cell_value(grid, hr, hc)
            if cell_value is not None and 0 <= cell_value < self.occupied_threshold:
                appearance_hits[(hr, hc)] += 1

            ray_cells = bresenham_cells(robot_rc[0], robot_rc[1], hr, hc)
            if len(ray_cells) >= 2:
                for rr, cc in ray_cells[:-1]:
                    v = self._cell_value(grid, rr, cc)
                    if v is not None and v >= self.occupied_threshold:
                        disappearance_hits[(rr, cc)] += 1

        self._update_and_publish(robot_id, grid, robot_pose_in_grid, appearance_hits, disappearance_hits)

    # ----------------------- core logic -----------------------

    def _robot_pose_in_grid(
        self,
        robot_id: int,
        grid_frame: str,
        base_frame: str,
        pose_msg: Optional[PoseStamped],
        pose_frame: str,
    ) -> Optional[PoseStamped]:
        T_grid_from_base = transform_matrix_from_tf(self.tf_buffer, grid_frame, base_frame)
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
                    (
                        float(pose_msg.pose.position.x),
                        float(pose_msg.pose.position.y),
                        float(pose_msg.pose.position.z),
                    ),
                )
                p.pose.position.x = x
                p.pose.position.y = y
                p.pose.position.z = z
                p.pose.orientation = deepcopy(pose_msg.pose.orientation)
                return p

        return deepcopy(pose_msg)

    def _update_and_publish(
        self,
        robot_id: int,
        grid: OccupancyGrid,
        robot_pose: PoseStamped,
        appearance_hits: Dict[Tuple[int, int], int],
        disappearance_hits: Dict[Tuple[int, int], int],
    ) -> None:
        now_sec = float(self.get_clock().now().nanoseconds) / 1e9

        self._update_streaks_and_publish_type(
            robot_id=robot_id,
            grid=grid,
            robot_pose=robot_pose,
            hits=appearance_hits,
            streaks=self.appearance_streak[robot_id],
            anomaly_type="appearance",
            now_sec=now_sec,
        )
        self._update_streaks_and_publish_type(
            robot_id=robot_id,
            grid=grid,
            robot_pose=robot_pose,
            hits=disappearance_hits,
            streaks=self.disappearance_streak[robot_id],
            anomaly_type="disappearance",
            now_sec=now_sec,
        )

    def _update_streaks_and_publish_type(
        self,
        robot_id: int,
        grid: OccupancyGrid,
        robot_pose: PoseStamped,
        hits: Dict[Tuple[int, int], int],
        streaks: Dict[Tuple[int, int], int],
        anomaly_type: str,
        now_sec: float,
    ) -> None:
        stale_keys = set(streaks.keys()) - set(hits.keys())
        for key in stale_keys:
            streaks[key] = 0

        for cell, evidence_count in hits.items():
            if evidence_count < self.min_points_per_cell:
                streaks[cell] = 0
                continue

            streaks[cell] = int(streaks.get(cell, 0)) + 1

            if streaks[cell] < self.stable_frames:
                continue

            cooldown_key = (robot_id, anomaly_type, cell[0], cell[1])
            last_t = self.last_published.get(cooldown_key, -1e9)
            if now_sec - last_t < self.cooldown_sec:
                continue

            event_x = float(grid.info.origin.position.x + (cell[1] + 0.5) * grid.info.resolution)
            event_y = float(grid.info.origin.position.y + (cell[0] + 0.5) * grid.info.resolution)

            event = AnomalyEvent(
                robot_id=robot_id,
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

    # ----------------------- helpers -----------------------

    def _cell_value(self, grid: OccupancyGrid, r: int, c: int) -> Optional[int]:
        if r < 0 or c < 0 or r >= grid.info.height or c >= grid.info.width:
            return None
        idx = r * grid.info.width + c
        if idx < 0 or idx >= len(grid.data):
            return None
        return int(grid.data[idx])

    def _world_to_cell(self, grid: OccupancyGrid, x: float, y: float) -> Optional[Tuple[int, int]]:
        res = float(grid.info.resolution)
        ox = float(grid.info.origin.position.x)
        oy = float(grid.info.origin.position.y)
        c = int(math.floor((x - ox) / res))
        r = int(math.floor((y - oy) / res))
        if 0 <= r < grid.info.height and 0 <= c < grid.info.width:
            return r, c
        return None

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
        self.get_logger().warn(
            f"Anomaly detected: robot={event.robot_id} type={event.anomaly_type} cell=({event.cell_r},{event.cell_c}) id={event.anomaly_id}"
        )


def main() -> None:
    rclpy.init()
    node = AnomalyDetectionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
