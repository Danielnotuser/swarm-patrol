#!/usr/bin/env python3
from __future__ import annotations

import math
import os
import re
import sys
import threading
import traceback
import time
from copy import deepcopy
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple
from collections import defaultdict

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from nav_msgs.msg import OccupancyGrid, Path as NavPath
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Header, Float32MultiArray
from visualization_msgs.msg import Marker

from ament_index_python.packages import get_package_share_directory
from tf2_ros import Buffer, TransformListener

#from cslam_common_interfaces.msg import PoseGraph
from darp_areas.msg import WakeUp

#os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
#os.environ.setdefault("SDL_AUDIODRIVER", "dummy")

package_name = "darp_areas"
package_share = get_package_share_directory(package_name)
DARP_SRC = Path(package_share) / "src"
sys.path.insert(0, str(DARP_SRC))

from multiRobotPathPlanner import MultiRobotPathPlanner


def parse_robot_index(ns: str) -> Optional[int]:
    """
    Expected names:
      keypoints_robot0
      keypoints_robot1
      ...
    """
    m = re.search(r"robot(\d+)", ns or "")
    return int(m.group(1)) if m else None


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


def yaw_from_quat(q: Quaternion) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def quat_from_yaw(yaw: float) -> Quaternion:
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def linear_to_rc(idx: int, cols: int) -> Tuple[int, int]:
    return idx // cols, idx % cols


def rc_to_linear(r: int, c: int, cols: int) -> int:
    return r * cols + c


@dataclass
class RasterGrid:
    occupancy: np.ndarray  # uint8: 0 free, 100 occupied
    origin_x: float
    origin_y: float
    resolution: float
    frame_id: str


class DarpBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("darp_node")
        self.cb_group = ReentrantCallbackGroup()

        self.declare_parameter("robot_count", 2)
        self.declare_parameter("robot_prefix", "r")
        self.declare_parameter("marker_topic", "/cslam/viz/cloudmarker")
        self.declare_parameter("pose_graph_topic", "/cslam/viz/pose_graph")
        self.declare_parameter("frame_id", "robot0_map")
        self.declare_parameter("default_resolution", 0.05)
        self.declare_parameter("default_padding", 1.0)
        self.declare_parameter("default_obstacle_dilation", 0)
        self.declare_parameter("obstacle_threshold", 0)
        self.declare_parameter("publish_unknown_outside", False)
        self.declare_parameter("visualize_darp", False)

        self.robot_count = int(self.get_parameter("robot_count").value)
        self.robot_prefix = str(self.get_parameter("robot_prefix").value)
        self.marker_topic = str(self.get_parameter("marker_topic").value)
        self.pose_graph_topic = str(self.get_parameter("pose_graph_topic").value)
        self.default_frame_id = str(self.get_parameter("frame_id").value)
        self.default_resolution = float(self.get_parameter("default_resolution").value)
        self.default_padding = float(self.get_parameter("default_padding").value)
        self.default_obstacle_dilation = int(self.get_parameter("default_obstacle_dilation").value)
        self.obstacle_threshold = float(self.get_parameter("obstacle_threshold").value)
        self.publish_unknown_outside = bool(self.get_parameter("publish_unknown_outside").value)
        self.visualize_darp = bool(self.get_parameter("visualize_darp").value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.accumulated_markers: Dict[int, List[Marker]] = defaultdict(list)
        self.latest_robot_pose: List[Optional[Pose]] = [None] * self.robot_count
        self._pose_timestamps: List[float] = [0.0] * self.robot_count
        self.latest_lock = threading.Lock()

        self.min_x = 1000
        self.max_x = -1000
        self.min_y = 1000
        self.max_y = -1000

        self.run_lock = threading.Lock()
        self.worker_thread: Optional[threading.Thread] = None

        self.marker_sub = self.create_subscription(
            Marker,
            self.marker_topic,
            self._marker_cb,
            10,
            callback_group=self.cb_group,
        )

        #self.pose_graph_sub = self.create_subscription(
        #    PoseGraph,
        #    self.pose_graph_topic,
        #    self._pose_graph_cb,
        #    10,
        #    callback_group=self.cb_group,
        #)

        self.wake_sub = self.create_subscription(
            WakeUp,
            "/darp/wake_up",
            self._wake_up_topic_cb,
            10,
            callback_group=self.cb_group,
        )

        frontier_grid_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )
        self.frontier_grid_sub = self.create_subscription(
            OccupancyGrid,
            "/frontier/grid",
            self._frontier_grid_cb,
            frontier_grid_qos,
            callback_group=self.cb_group,
        )
        self.latest_frontier_grid: Optional[OccupancyGrid] = None

        self.frontier_status_sub = self.create_subscription(
            Float32MultiArray,
            "/frontier/frontier_status",
            self._frontier_status_cb,
            10,
            callback_group=self.cb_group,
        )

        self.area_pubs = []
        self.route_pubs = []
        self.full_area_pubs = []
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        for i in range(self.robot_count):
            ns = f"{self.robot_prefix}{i}"
            self.area_pubs.append(
                self.create_publisher(OccupancyGrid, f"/{ns}/darp/area", qos)
            )
            self.route_pubs.append(
                self.create_publisher(NavPath, f"/{ns}/darp/route", qos)
            )
            self.full_area_pubs.append(
                self.create_publisher(OccupancyGrid, f"/{ns}/darp/full_area", qos)
            )

        self.get_logger().info(
            f"Listening to {self.marker_topic} and {self.pose_graph_topic}. "
            f"Listening on /darp/wake_up (topic)."
        )

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

    def _transform_pose_to_frame(self, pose: Pose, source_frame: str, target_frame: str) -> Pose:
        if not source_frame or source_frame == target_frame:
            return deepcopy(pose)

        if not self.tf_buffer.can_transform(target_frame, source_frame, Time(seconds=0), timeout=rclpy.duration.Duration(seconds=1.0)):
            self.get_logger().warn(
                f"TF not ready yet: {source_frame} -> {target_frame}"
            )
            return None

        T = self._lookup_transform_matrix(target_frame, source_frame)
        v = np.array([pose.position.x, pose.position.y, pose.position.z, 1.0], dtype=np.float64)
        w = T @ v

        out = deepcopy(pose)
        out.position.x = float(w[0])
        out.position.y = float(w[1])
        out.position.z = float(w[2])
        return out

    def _marker_points_in_frame(self, marker: Marker, target_frame: str) -> np.ndarray:
        if not marker.points:
            return np.zeros((0, 2), dtype=np.float64)

        source_frame = marker.header.frame_id or target_frame

        if not self.tf_buffer.can_transform(target_frame, source_frame, Time(seconds=0), timeout=rclpy.duration.Duration(seconds=3.0)):
            self.get_logger().warn(
                f"TF not ready yet: {source_frame} -> {target_frame}"
            )
            return None

        try:
            T = self._lookup_transform_matrix(target_frame, source_frame)
        except Exception as exc:
            self.get_logger().warn(
                f"TF lookup failed {source_frame} -> {target_frame}: {exc}. "
                f"Skipping this marker."
            )
            return np.zeros((0, 2), dtype=np.float64)

        pts = []

        for p in marker.points:
            v = np.array([p.x, p.y, p.z, 1.0], dtype=np.float64)
            w = T @ v

            if w[0] < self.min_x: self.min_x = w[0]
            if w[1] < self.min_y: self.min_y = w[1]
            if w[0] > self.max_x: self.max_x = w[0]
            if w[1] > self.max_y: self.max_y = w[1]

            # Считаем препятствием только то, что выше порога по z
            if w[2] < self.obstacle_threshold:
                continue

            pts.append((float(w[0]), float(w[1])))

        return np.asarray(pts, dtype=np.float64)

    # ------------------------ ROS callbacks ------------------------

    def _frontier_grid_cb(self, msg: OccupancyGrid) -> None:
        with self.latest_lock:
            self.latest_frontier_grid = msg
        self.get_logger().info(
            f"Received frontier grid: {msg.info.width}x{msg.info.height}, "
            f"res={msg.info.resolution}, frame={msg.header.frame_id}"
        )

    def _frontier_status_cb(self, msg: Float32MultiArray) -> None:
        if len(msg.data) < 4:
            return
        rid = int(msg.data[0])
        if not (0 <= rid < self.robot_count):
            return
        pose = Pose()
        pose.position.x = float(msg.data[2])
        pose.position.y = float(msg.data[3])
        pose.position.z = 0.0
        pose.orientation.w = 1.0
        with self.latest_lock:
            self.latest_robot_pose[rid] = pose
            self._pose_timestamps[rid] = time.time()

    def _marker_cb(self, msg: Marker) -> None:
        robot_idx = parse_robot_index(msg.ns)
        if robot_idx is None:
            return
        if not (0 <= robot_idx < self.robot_count):
            self.get_logger().warn(f"Ignoring marker with out-of-range robot index: {robot_idx}")
            return

        with self.latest_lock:
            self.accumulated_markers[robot_idx].append(msg)

    # ------------------------ PoseGraph handling (commented — unused, using frontier_status instead) ------------------------

    #def _extract_latest_pose_from_pose_graph(self, msg: PoseGraph) -> tuple[Optional[Pose], Optional[str]]:
    #    values = list(getattr(msg, "values", []))
    #    if values:
    #        latest = max(values, key=lambda v: int(v.key.keyframe_id))
    #        return deepcopy(latest.pose), f"robot{latest.key.robot_id}_map"
    #    else:
    #        return None, None
    #    #edges = list(getattr(msg, "edges", []))
    #    #if not edges:
    #    #    return None, None
    #
    #    #edges = sorted(
    #    #    edges,
    #    #    key=lambda e: (int(e.key_from.keyframe_id), int(e.key_to.keyframe_id)),
    #    #)
    #
    #    #x, y, yaw = 0.0, 0.0, 0.0
    #    #for e in edges:
    #    #    dx = float(e.measurement.position.x)
    #    #    dy = float(e.measurement.position.y)
    #    #    dyaw = yaw_from_quat(e.measurement.orientation)
    #    #    c = math.cos(yaw)
    #    #    s = math.sin(yaw)
    #    #    x = x + c * dx - s * dy
    #    #    y = y + s * dx + c * dy
    #    #    yaw = yaw + dyaw
    #
    #    #pose = Pose()
    #    #pose.position.x = x
    #    #pose.position.y = y
    #    #pose.position.z = 0.0
    #    #pose.orientation = quat_from_yaw(yaw)
    #    #return pose, frame_id

    #def _pose_graph_cb(self, msg: PoseGraph) -> None:
    #    robot_idx = int(msg.robot_id)
    #    if not (0 <= robot_idx < self.robot_count):
    #        return
    #
    #    pose, frame_id = self._extract_latest_pose_from_pose_graph(msg)
    #    if pose is None or frame_id is None:
    #        return
    #
    #    transformed_pose = self._transform_pose_to_frame(pose, frame_id, self.default_frame_id)
    #
    #    with self.latest_lock:
    #        if self.latest_robot_pose[robot_idx] != transformed_pose:
    #            self.get_logger().info(
    #                f"PoseGraph robot {robot_idx}: latest pose stored in {self.default_frame_id} "
    #                f"= ({transformed_pose.position.x:.3f}, {transformed_pose.position.y:.3f})"
    #            )
    #        self.latest_robot_pose[robot_idx] = transformed_pose


    # ------------------------ Wake-up (topic) ------------------------

    def _wake_up_topic_cb(self, msg: WakeUp) -> None:
        active_ids = list(msg.active_robot_ids)
        if not active_ids:
            self.get_logger().warn("Empty active_robot_ids, ignoring.")
            return

        with self.run_lock:
            if self.worker_thread is not None and self.worker_thread.is_alive():
                self.get_logger().warn("DARP already running, ignoring wake-up request.")
                return

            deadline = time.time() + 7.0
            while time.time() < deadline:
                with self.latest_lock:
                    poses_copy = deepcopy(self.latest_robot_pose)
                    timestamps_copy = list(self._pose_timestamps)
                now = time.time()
                all_fresh = True
                for rid in active_ids:
                    if rid >= len(poses_copy) or poses_copy[rid] is None:
                        all_fresh = False
                        break
                    if now - timestamps_copy[rid] > 0.5:
                        all_fresh = False
                        break
                if all_fresh:
                    break
                time.sleep(0.1)

            missing_poses = [rid for rid in active_ids if rid >= len(poses_copy) or poses_copy[rid] is None]

            if missing_poses:
                self.get_logger().error(f"Missing poses for robots: {missing_poses} after 1s timeout")
                return

            for rid in active_ids:
                p = poses_copy[rid]
                age = now - timestamps_copy[rid]
                self.get_logger().info(f'DARP start robot {rid}: ({p.position.x:.2f}, {p.position.y:.2f}), age={age:.2f}s')

            with self.latest_lock:
                markers_copy = deepcopy(self.accumulated_markers)
                has_frontier_grid = self.latest_frontier_grid is not None

            if not has_frontier_grid:
                missing_markers = [i for i in active_ids if i not in markers_copy]
                if missing_markers:
                    self.get_logger().error(f"Missing cloudmarker for robots: {missing_markers}")
                    return

            self.worker_thread = threading.Thread(
                target=self._run_darp_job,
                args=(poses_copy, msg, active_ids),
                daemon=True,
            )
            self.worker_thread.start()

        self.get_logger().info(f"DARP started for robots: {active_ids}")

    # ------------------------ DARP job ------------------------

    def _run_darp_job(
        self,
        poses: List[Optional[Pose]],
        msg: WakeUp,
        active_ids: List[int],
    ) -> None:
        try:
            n_robots = len(active_ids)
            with self.latest_lock:
                frontier_grid = self.latest_frontier_grid
            if frontier_grid is not None:
                self.get_logger().info(
                    "DARP running in frontier grid"
                )
                grid = self._build_from_frontier_grid(frontier_grid, msg)
            else:
                self.get_logger().info(
                    "DARP running in markers grid"
                )
                grid = self._build_raster_grid(msg)
            start_cells = self._resolve_start_cells(grid, poses, active_ids)
            portions = self._resolve_portions(msg, n_robots)

            obs_pos = np.flatnonzero(grid.occupancy.reshape(-1) > 0).astype(int).tolist()

            self.get_logger().info(
                f"Grid {grid.occupancy.shape[0]}x{grid.occupancy.shape[1]}, "
                f"robots={n_robots}, obstacles={len(obs_pos)}"
            )
            self.get_logger().info(f"Start cells: {start_cells}")
            self.get_logger().info(f"Portions: {portions}")

            planner = MultiRobotPathPlanner(
                grid.occupancy.shape[0],
                grid.occupancy.shape[1],
                not bool(msg.use_equal_portions),
                start_cells,
                portions,
                obs_pos,
                self.visualize_darp,
            )

            if not getattr(planner, "DARP_success", False):
                raise RuntimeError("DARP did not find a feasible partition.")

            self._publish_result(grid, planner, start_cells, active_ids)
            self.get_logger().info("DARP finished and topics were published.")

        except Exception as exc:
            self.get_logger().error(f"DARP run failed: {exc}")
            self.get_logger().error(traceback.format_exc())

    # ------------------------ Grid build ------------------------

    def _build_from_frontier_grid(
        self,
        frontier_grid: OccupancyGrid,
        msg: WakeUp,
    ) -> RasterGrid:
        resolution = frontier_grid.info.resolution
        width = frontier_grid.info.width
        height = frontier_grid.info.height
        origin_x = frontier_grid.info.origin.position.x
        origin_y = frontier_grid.info.origin.position.y

        target_resolution = msg.resolution
        obstacle_dilation = msg.obstacle_dilation
        padding_m = float(msg.padding) if msg.padding >= 0.0 else self.default_padding

        occupancy = np.array(frontier_grid.data, dtype=np.int8).reshape(height, width)

        unknown = int(np.sum(occupancy == -1))
        free = int(np.sum(occupancy == 0))
        occ = int(np.sum(occupancy == 100))
        self.get_logger().info(
            f"Frontier grid: {width}x{height}, res={resolution}, "
            f"unknown={unknown}, free={free}, occupied={occ}"
        )

        un_mask = (occupancy == -1).astype(np.float32)
        fr_mask = (occupancy == 0).astype(np.float32)

        kernel = np.ones((3, 3), dtype=np.float32)
        kernel[1, 1] = 0

        n_unknown = cv2.filter2D(un_mask, -1, kernel, borderType=cv2.BORDER_CONSTANT)
        n_free = cv2.filter2D(fr_mask, -1, kernel, borderType=cv2.BORDER_CONSTANT)

        occupancy[(n_unknown == 0) & un_mask.astype(bool)] = 0
        occupancy[(n_free == 0) & un_mask.astype(bool)] = 100

        if abs(resolution - target_resolution) > 1e-6:
            tgt_width = int(round(width * resolution / target_resolution))
            tgt_height = int(round(height * resolution / target_resolution))

            occ_mask = (occupancy == 100).astype(np.uint8) * 255
            unk_mask = (occupancy == -1).astype(np.uint8) * 255

            down_occ = cv2.resize(occ_mask, (tgt_width, tgt_height), interpolation=cv2.INTER_AREA)
            down_unk = cv2.resize(unk_mask, (tgt_width, tgt_height), interpolation=cv2.INTER_AREA)

            occupancy = np.full((tgt_height, tgt_width), 0, dtype=np.int8)
            occupancy[down_occ > 0] = 100
            occupancy[down_unk > 0] = 100

            width = tgt_width
            height = tgt_height
            resolution = target_resolution
            origin_x = frontier_grid.info.origin.position.x
            origin_y = frontier_grid.info.origin.position.y

            self.get_logger().info(
                f"Downsampled to: {width}x{height}, res={resolution}"
            )

        occupancy[occupancy != 100] = 0

        dil_cells = obstacle_dilation if obstacle_dilation >= 0 else self.default_obstacle_dilation
        if dil_cells > 0:
            kernel = np.ones((2 * dil_cells + 1, 2 * dil_cells + 1), dtype=np.uint8)
            occ_bin = (occupancy > 0).astype(np.uint8) * 255
            occ_bin = cv2.dilate(occ_bin, kernel, iterations=1)
            original_mask = (occupancy == 100)
            occupancy = np.where(occ_bin > 0, 100, 0).astype(np.int8)
            occupancy[(occ_bin > 0) & (~original_mask)] = 50

        padding_cells = max(0, int(padding_m / resolution))
        padded = np.full(
            (height + 2 * padding_cells, width + 2 * padding_cells),
            100,
            dtype=np.uint8,
        )
        padded[padding_cells:padding_cells + height, padding_cells:padding_cells + width] = occupancy.astype(np.uint8)

        occ = padded
        origin_x -= padding_cells * resolution
        origin_y -= padding_cells * resolution

        free_mask = (occ == 0).astype(np.uint8)
        num_labels, labels = cv2.connectedComponents(free_mask, connectivity=4)
        if num_labels > 2:
            counts = np.bincount(labels.reshape(-1))
            counts[0] = 0
            keep = int(np.argmax(counts))
            free_keep = labels == keep
            occ[~free_keep] = 100

        frame_id = frontier_grid.header.frame_id or self.default_frame_id
        return RasterGrid(
            occupancy=occ,
            origin_x=origin_x,
            origin_y=origin_y,
            resolution=resolution,
            frame_id=frame_id,
        )

    def _build_raster_grid(
        self,
        msg: WakeUp,
    ) -> RasterGrid:
        resolution = float(msg.resolution) if msg.resolution > 0.0 else self.default_resolution
        padding = float(msg.padding) if msg.padding >= 0.0 else self.default_padding

        all_pts = []
        for chunk in self.accumulated_markers.values():
            for marker in chunk:
                pts = self._marker_points_in_frame(marker, self.default_frame_id)
                if pts.shape[0] > 0:
                    all_pts.append(pts)

        if not all_pts:
            raise RuntimeError("No usable points in received markers.")

        points_xy = np.vstack(all_pts)

        min_x = float(self.min_x - padding)
        max_x = float(self.max_x + padding)
        min_y = float(self.min_y - padding)
        max_y = float(self.max_y + padding)

        cols = max(1, int(math.ceil((max_x - min_x) / resolution)) + 1)
        rows = max(1, int(math.ceil((max_y - min_y) / resolution)) + 1)

        occ = np.zeros((rows, cols), dtype=np.uint8)

        for x, y in points_xy:
            c = int((x - min_x) / resolution)
            r = int((y - min_y) / resolution)
            if 0 <= r < rows and 0 <= c < cols:
                occ[r, c] = 100

        dil_cells = msg.obstacle_dilation if msg.obstacle_dilation >= 0 else self.default_obstacle_dilation
        if dil_cells > 0:
            kernel = np.ones((2 * dil_cells + 1, 2 * dil_cells + 1), dtype=np.uint8)
            occ_bin = (occ > 0).astype(np.uint8) * 255
            occ_bin = cv2.dilate(occ_bin, kernel, iterations=1)
            original_mask = (occ == 100)
            occ = np.where(occ_bin > 0, 100, 0).astype(np.uint8)
            occ[(occ_bin > 0) & (~original_mask)] = 50

        #print("Here is occupancy grid: ", occ)

        free_mask = (occ == 0).astype(np.uint8)
        num_labels, labels = cv2.connectedComponents(free_mask, connectivity=4)
        if num_labels > 2:
            counts = np.bincount(labels.reshape(-1))
            counts[0] = 0
            keep = int(np.argmax(counts))
            free_keep = labels == keep
            occ[~free_keep] = 100

        frame_id = self.default_frame_id
        return RasterGrid(
            occupancy=occ,
            origin_x=min_x,
            origin_y=min_y,
            resolution=resolution,
            frame_id=frame_id,
        )

    # ------------------------ Portions / starts ------------------------

    def _resolve_portions(self, msg: WakeUp, n_robots: int) -> List[float]:
        if bool(msg.use_equal_portions):
            return [1.0 / n_robots] * n_robots

        portions = list(msg.portions)
        if len(portions) != n_robots:
            raise RuntimeError(
                f"portions length must be {n_robots}, got {len(portions)}"
            )
        total = float(sum(portions))
        if abs(total - 1.0) > 1e-4:
            raise RuntimeError(f"portions must sum to 1.0, got {total}")
        return portions

    def _resolve_start_cells(
            self,
            grid: RasterGrid,
            poses: List[Optional[Pose]],
            active_ids: List[int],
    ) -> List[int]:
        starts: List[int] = []

        for i in active_ids:
            pose = poses[i]
            if pose is None:
                raise RuntimeError(f"Missing pose for robot {i}")

            x = float(pose.position.x)
            y = float(pose.position.y)

            c = int((x - grid.origin_x) / grid.resolution)
            r = int((y - grid.origin_y) / grid.resolution)

            if not (0 <= r < grid.occupancy.shape[0] and 0 <= c < grid.occupancy.shape[1]):
                raise RuntimeError(
                    f"Robot {i} pose ({x:.3f}, {y:.3f}) is outside the occupancy grid."
                )

            if grid.occupancy[r, c] > 0:
                free_cells = np.argwhere(grid.occupancy == 0)
                if free_cells.size == 0:
                    raise RuntimeError("No free cells available in the grid.")
                d2 = np.sum((free_cells - np.array([r, c])) ** 2, axis=1)
                nearest = free_cells[int(np.argmin(d2))]
                r, c = int(nearest[0]), int(nearest[1])
                self.get_logger().warn(
                    f"Robot {i} start was occupied; snapped to nearest free cell ({r}, {c})."
                )

            starts.append(rc_to_linear(r, c, grid.occupancy.shape[1]))

        return starts

    # ------------------------ Publishing ------------------------

    def _publish_result(self, grid: RasterGrid, planner: object, start_cells: List[int], active_ids: List[int]) -> None:
        assignment = np.array(planner.darp_instance.A, dtype=np.int32)

        for idx, robot_idx in enumerate(active_ids):
            robot_frame_id = f"robot{robot_idx}_map"
            area_msg = self._make_area_msg(grid, assignment, idx, robot_frame_id)
            route_msg = self._make_route_msg(grid, planner.best_case.paths[idx], start_cells[idx], robot_frame_id)
            full_area_msg = self._make_full_area_msg(grid, robot_frame_id)
            self.area_pubs[robot_idx].publish(area_msg)
            self.route_pubs[robot_idx].publish(route_msg)
            self.full_area_pubs[robot_idx].publish(full_area_msg)

    def _make_area_msg(
        self,
        grid: RasterGrid,
        assignment: np.ndarray,
        robot_idx: int,
        frame_id: str,
    ) -> OccupancyGrid:
        msg = OccupancyGrid()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id

        msg.info.resolution = float(grid.resolution)
        msg.info.width = int(grid.occupancy.shape[1])
        msg.info.height = int(grid.occupancy.shape[0])
        msg.info.origin.position.x = float(grid.origin_x)
        msg.info.origin.position.y = float(grid.origin_y)
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0

        data = np.zeros_like(grid.occupancy, dtype=np.int8)
        data[assignment != robot_idx] = 100

        if self.publish_unknown_outside:
            pass

        msg.data = data.reshape(-1).tolist()
        return msg

    def _make_full_area_msg(
        self,
        grid: RasterGrid,
        frame_id: str,
    ) -> OccupancyGrid:
        msg = OccupancyGrid()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id

        msg.info.resolution = float(grid.resolution)
        msg.info.width = int(grid.occupancy.shape[1])
        msg.info.height = int(grid.occupancy.shape[0])
        msg.info.origin.position.x = float(grid.origin_x)
        msg.info.origin.position.y = float(grid.origin_y)
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0

        msg.data = grid.occupancy.reshape(-1).tolist()
        return msg

    def _path_sequence_to_vertices(
            self,
            path_sequence: Sequence[Tuple[int, int, int, int]],
    ) -> List[Tuple[int, int]]:
        if not path_sequence:
            return []

        vertices: List[Tuple[int, int]] = [
            (int(path_sequence[0][0]), int(path_sequence[0][1]))
        ]

        for seg in path_sequence:
            b = (int(seg[2]), int(seg[3]))
            if b != vertices[-1]:
                vertices.append(b)

        return vertices

    def _make_route_msg(
            self,
            grid: RasterGrid,
            path_sequence: Sequence[Tuple[int, int, int, int]],
            start_cell: int,
            frame_id: str,
    ) -> NavPath:
        msg = NavPath()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id

        # Разрешение для удвоенной STC-сетки
        route_resolution = grid.resolution / 2.0

        def indices_to_xy(rr: int, cc: int) -> tuple[float, float]:
            x = grid.origin_x + (cc + 0.5) * route_resolution
            y = grid.origin_y + (rr + 0.5) * route_resolution
            return x, y

        start_r, start_c = linear_to_rc(start_cell, grid.occupancy.shape[1])
        start_rc = (2 * start_r, 2 * start_c)

        route_vertices = self._path_sequence_to_vertices(path_sequence)
        if not route_vertices:
            # пустой путь → только старт
            p = PoseStamped()
            p.header.frame_id = frame_id
            p.header.stamp = msg.header.stamp
            p.pose.position.x, p.pose.position.y = indices_to_xy(*start_rc)
            p.pose.position.z = 0.0
            p.pose.orientation.w = 1.0
            msg.poses.append(p)
            return msg

        # Определяем ближайшую точку маршрута к старту и циклически сдвигаем
        if start_rc in route_vertices:
            start_idx = route_vertices.index(start_rc)
        else:
            start_idx = min(
                range(len(route_vertices)),
                key=lambda i: (route_vertices[i][0] - start_rc[0]) ** 2 +
                              (route_vertices[i][1] - start_rc[1]) ** 2,
            )
            start_rc = route_vertices[start_idx]

        rotated = route_vertices[start_idx:] + route_vertices[:start_idx]

        # --- Фильтрация: оставляем точки с шагом не более max_step и не чаще min_step ---
        max_step_m = 1.0  # максимально допустимое расстояние между точками пути
        min_step_m = 0.05  # минимальное (чтобы избежать дублей)
        #max_points = 800  # ограничение общего числа точек

        filtered_vertices = [start_rc]
        prev_x, prev_y = indices_to_xy(*start_rc)

        for rr, cc in rotated:
            if (rr, cc) == start_rc:
                continue
            x, y = indices_to_xy(rr, cc)
            dist = math.hypot(x - prev_x, y - prev_y)
            if dist < min_step_m:
                continue
            if dist > max_step_m:
                # Вставляем промежуточные точки по прямой (линейная интерполяция)
                steps = int(math.ceil(dist / max_step_m))
                for s in range(1, steps):
                    frac = s / steps
                    interp_x = prev_x + (x - prev_x) * frac
                    interp_y = prev_y + (y - prev_y) * frac
                    # Добавляем только если не дублирует предыдущую
                    if math.hypot(interp_x - prev_x, interp_y - prev_y) > min_step_m:
                        p = PoseStamped()
                        p.header.frame_id = frame_id
                        p.header.stamp = msg.header.stamp
                        p.pose.position.x = interp_x
                        p.pose.position.y = interp_y
                        p.pose.position.z = 0.0
                        p.pose.orientation.w = 1.0
                        msg.poses.append(p)
                        prev_x, prev_y = interp_x, interp_y

            filtered_vertices.append((rr, cc))
            p = PoseStamped()
            p.header.frame_id = frame_id
            p.header.stamp = msg.header.stamp
            p.pose.position.x = x
            p.pose.position.y = y
            p.pose.position.z = 0.0
            p.pose.orientation.w = 1.0
            msg.poses.append(p)
            prev_x, prev_y = x, y

            #if len(msg.poses) >= max_points:
            #    self.get_logger().warn(f"Route truncated to {max_points} points")
            #    break

        # Если получилось слишком мало точек (например, только старт), добавляем хотя бы одну целевую
        if len(msg.poses) < 2:
            # добавляем последнюю точку маршрута
            last_r, last_c = rotated[-1]
            x, y = indices_to_xy(last_r, last_c)
            p = PoseStamped()
            p.header.frame_id = frame_id
            p.header.stamp = msg.header.stamp
            p.pose.position.x = x
            p.pose.position.y = y
            p.pose.position.z = 0.0
            p.pose.orientation.w = 1.0
            msg.poses.append(p)

        return msg


def main() -> None:
    rclpy.init()
    node = DarpBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()