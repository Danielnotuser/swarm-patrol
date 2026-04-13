#!/usr/bin/env python3
from __future__ import annotations

import math
import os
import re
import sys
import threading
import traceback
from copy import deepcopy
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from nav_msgs.msg import OccupancyGrid, Path as NavPath
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from std_msgs.msg import Header
from visualization_msgs.msg import Marker

from pathlib import Path
import sys
from ament_index_python.packages import get_package_share_directory

DARP_SRC = Path(get_package_share_directory("darp")) / "src"
sys.path.insert(0, str(DARP_SRC))

from multiRobotPathPlanner import MultiRobotPathPlanner
from darp.srv import WakeUp

os.environ.setdefault("SDL_VIDEODRIVER", "dummy")

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


def transform_marker_points(marker: Marker) -> np.ndarray:
    if not marker.points:
        return np.zeros((0, 2), dtype=np.float64)

    R = quat_to_rot_matrix(marker.pose.orientation)
    t = np.array(
        [marker.pose.position.x, marker.pose.position.y, marker.pose.position.z],
        dtype=np.float64,
    )

    pts = []
    for p in marker.points:
        v = np.array([p.x, p.y, p.z], dtype=np.float64)
        w = (R @ v) + t
        pts.append((float(w[0]), float(w[1])))
    return np.asarray(pts, dtype=np.float64)


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

        self.declare_parameter("robot_count", 3)
        self.declare_parameter("robot_prefix", "r")
        self.declare_parameter("marker_topic", "/cslam/viz/cloudmaker")
        self.declare_parameter("frame_id", "robot0_map")
        self.declare_parameter("default_resolution", 0.05)
        self.declare_parameter("default_padding", 1.0)
        self.declare_parameter("default_obstacle_dilation", 0.10)
        self.declare_parameter("sleep_after_run", True)
        self.declare_parameter("publish_unknown_outside", False)

        self.robot_count = int(self.get_parameter("robot_count").value)
        self.robot_prefix = str(self.get_parameter("robot_prefix").value)
        self.marker_topic = str(self.get_parameter("marker_topic").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.default_resolution = float(self.get_parameter("default_resolution").value)
        self.default_padding = float(self.get_parameter("default_padding").value)
        self.default_obstacle_dilation = float(self.get_parameter("default_obstacle_dilation").value)
        self.sleep_after_run = bool(self.get_parameter("sleep_after_run").value)
        self.publish_unknown_outside = bool(self.get_parameter("publish_unknown_outside").value)

        self.latest_robot_pose: List[Optional[Pose]] = [None] * self.robot_count
        self.latest_robot_marker: Dict[int, Marker] = {}
        self.latest_lock = threading.Lock()

        self.run_lock = threading.Lock()
        self.worker_thread: Optional[threading.Thread] = None
        self.asleep = True

        self.marker_sub = self.create_subscription(
            Marker,
            self.marker_topic,
            self._marker_cb,
            10,
            callback_group=self.cb_group,
        )
        self.wake_srv = self.create_service(
            WakeUp,
            "/darp/wake_up",
            self._wake_up_cb,
            callback_group=self.cb_group,
        )

        self.area_pubs = []
        self.route_pubs = []
        for i in range(self.robot_count):
            ns = f"{self.robot_prefix}{i}"
            self.area_pubs.append(
                self.create_publisher(OccupancyGrid, f"/{ns}/darp/area", 10)
            )
            self.route_pubs.append(
                self.create_publisher(NavPath, f"/{ns}/darp/route", 10)
            )

        self.get_logger().info(
            f"Listening to {self.marker_topic}. Waiting for /darp/wake_up."
        )

    def _marker_cb(self, msg: Marker) -> None:
        robot_idx = parse_robot_index(msg.ns)
        if robot_idx is None:
            return
        if not (0 <= robot_idx < self.robot_count):
            self.get_logger().warn(f"Ignoring marker with out-of-range robot index: {robot_idx}")
            return

        with self.latest_lock:
            self.latest_robot_marker[robot_idx] = deepcopy(msg)
            self.latest_robot_pose[robot_idx] = deepcopy(msg.pose)

    def _wake_up_cb(self, request: WakeUp.Request, response: WakeUp.Response) -> WakeUp.Response:
        with self.run_lock:
            if self.worker_thread is not None and self.worker_thread.is_alive():
                response.success = False
                response.message = "DARP is already running."
                return response

            with self.latest_lock:
                has_all_poses = all(p is not None for p in self.latest_robot_pose)
                markers_copy = deepcopy(self.latest_robot_marker)
                poses_copy = deepcopy(self.latest_robot_pose)

            if not markers_copy:
                response.success = False
                response.message = "No cloudmaker markers received yet."
                return response

            if not has_all_poses:
                missing = [i for i, p in enumerate(poses_copy) if p is None]
                response.success = False
                response.message = f"Missing robot poses for robots: {missing}"
                return response

            self.asleep = False
            self.worker_thread = threading.Thread(
                target=self._run_darp_job,
                args=(markers_copy, poses_copy, request),
                daemon=True,
            )
            self.worker_thread.start()

        response.success = True
        response.message = "DARP started."
        return response

    def _run_darp_job(
        self,
        markers: Dict[int, Marker],
        poses: List[Optional[Pose]],
        request: WakeUp.Request,
    ) -> None:
        try:
            grid = self._build_raster_grid(markers, request)
            start_cells = self._resolve_start_cells(grid, poses)
            portions = self._resolve_portions(request)

            obs_pos = np.flatnonzero(grid.occupancy.reshape(-1) > 0).astype(int).tolist()

            self.get_logger().info(
                f"Grid {grid.occupancy.shape[0]}x{grid.occupancy.shape[1]}, "
                f"robots={self.robot_count}, obstacles={len(obs_pos)}"
            )
            self.get_logger().info(f"Start cells: {start_cells}")
            self.get_logger().info(f"Portions: {portions}")

            planner = MultiRobotPathPlanner(
                grid.occupancy.shape[0],
                grid.occupancy.shape[1],
                not bool(request.use_equal_portions) is False,
                start_cells,
                portions,
                obs_pos,
                True,
            )

            if not getattr(planner, "DARP_success", False):
                raise RuntimeError("DARP did not find a feasible partition.")

            self._publish_result(grid, planner)
            self.get_logger().info("DARP finished and topics were published.")

        except Exception as exc:
            self.get_logger().error(f"DARP run failed: {exc}")
            self.get_logger().error(traceback.format_exc())
        finally:
            if self.sleep_after_run:
                self.asleep = True

    def _build_raster_grid(
        self,
        markers: Dict[int, Marker],
        request: WakeUp.Request,
    ) -> RasterGrid:
        resolution = float(request.resolution) if request.resolution > 0.0 else self.default_resolution
        padding = float(request.padding) if request.padding >= 0.0 else self.default_padding
        obstacle_dilation = (
            float(request.obstacle_dilation)
            if request.obstacle_dilation >= 0.0
            else self.default_obstacle_dilation
        )

        all_pts = []
        for marker in markers.values():
            pts = transform_marker_points(marker)
            if pts.shape[0] > 0:
                all_pts.append(pts)

        if not all_pts:
            raise RuntimeError("No usable points in received markers.")

        points_xy = np.vstack(all_pts)

        min_x = float(np.min(points_xy[:, 0]) - padding)
        max_x = float(np.max(points_xy[:, 0]) + padding)
        min_y = float(np.min(points_xy[:, 1]) - padding)
        max_y = float(np.max(points_xy[:, 1]) + padding)

        cols = max(1, int(math.ceil((max_x - min_x) / resolution)) + 1)
        rows = max(1, int(math.ceil((max_y - min_y) / resolution)) + 1)

        occ = np.zeros((rows, cols), dtype=np.uint8)

        for x, y in points_xy:
            c = int((x - min_x) / resolution)
            r = int((y - min_y) / resolution)
            if 0 <= r < rows and 0 <= c < cols:
                occ[r, c] = 100

        dil_cells = int(round(obstacle_dilation / resolution))
        if dil_cells > 0:
            kernel = np.ones((2 * dil_cells + 1, 2 * dil_cells + 1), dtype=np.uint8)
            occ_bin = (occ > 0).astype(np.uint8) * 255
            occ_bin = cv2.dilate(occ_bin, kernel, iterations=1)
            occ = (occ_bin > 0).astype(np.uint8) * 100

        # Keep only the largest free component so DARP gets one connected free space.
        free_mask = (occ == 0).astype(np.uint8)
        num_labels, labels = cv2.connectedComponents(free_mask, connectivity=4)
        if num_labels > 2:
            counts = np.bincount(labels.reshape(-1))
            counts[0] = 0
            keep = int(np.argmax(counts))
            free_keep = labels == keep
            occ[~free_keep] = 100

        frame_id = next(iter(markers.values())).header.frame_id or self.frame_id
        return RasterGrid(
            occupancy=occ,
            origin_x=min_x,
            origin_y=min_y,
            resolution=resolution,
            frame_id=frame_id,
        )

    def _resolve_portions(self, request: WakeUp.Request) -> List[float]:
        if bool(request.use_equal_portions):
            return [1.0 / self.robot_count] * self.robot_count

        portions = list(request.portions)
        if len(portions) != self.robot_count:
            raise RuntimeError(
                f"portions length must be {self.robot_count}, got {len(portions)}"
            )
        total = float(sum(portions))
        if abs(total - 1.0) > 1e-4:
            raise RuntimeError(f"portions must sum to 1.0, got {total}")
        return portions

    def _resolve_start_cells(
        self,
        grid: RasterGrid,
        poses: List[Optional[Pose]],
    ) -> List[int]:
        starts: List[int] = []
        for i, pose in enumerate(poses):
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

    def _publish_result(self, grid: RasterGrid, planner: object) -> None:
        assignment = np.array(planner.darp_instance.A, dtype=np.int32)
        drone_no = int(planner.darp_instance.droneNo)

        for robot_idx in range(self.robot_count):
            area_msg = self._make_area_msg(grid, assignment, robot_idx, drone_no)
            route_msg = self._make_route_msg(grid, planner.best_case.paths[robot_idx])
            self.area_pubs[robot_idx].publish(area_msg)
            self.route_pubs[robot_idx].publish(route_msg)

    def _make_area_msg(
        self,
        grid: RasterGrid,
        assignment: np.ndarray,
        robot_idx: int,
        drone_no: int,
    ) -> OccupancyGrid:
        msg = OccupancyGrid()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = grid.frame_id

        msg.info.resolution = float(grid.resolution)
        msg.info.width = int(grid.occupancy.shape[1])
        msg.info.height = int(grid.occupancy.shape[0])
        msg.info.origin.position.x = float(grid.origin_x)
        msg.info.origin.position.y = float(grid.origin_y)
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0

        data = np.zeros_like(grid.occupancy, dtype=np.int8)
        data[assignment == robot_idx] = 100
        data[assignment == drone_no] = 100

        if self.publish_unknown_outside:
            # Keep the raster explicit: 0 free/unassigned, 100 occupied/assigned.
            pass

        msg.data = data.reshape(-1).tolist()
        return msg

    def _make_route_msg(
        self,
        grid: RasterGrid,
        path_sequence: Sequence[Tuple[int, int, int, int]],
    ) -> NavPath:
        msg = NavPath()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = grid.frame_id

        poses: List[PoseStamped] = []
        subcell = grid.resolution / 2.0

        def add_rc(rr: int, cc: int) -> None:
            p = PoseStamped()
            p.header.frame_id = grid.frame_id
            p.header.stamp = msg.header.stamp
            p.pose.position.x = float(grid.origin_x + (cc + 0.5) * subcell)
            p.pose.position.y = float(grid.origin_y + (rr + 0.5) * subcell)
            p.pose.position.z = 0.0
            p.pose.orientation.w = 1.0
            poses.append(p)

        last = None
        for seg in path_sequence:
            a = (int(seg[0]), int(seg[1]))
            b = (int(seg[2]), int(seg[3]))
            if last != a:
                add_rc(*a)
            add_rc(*b)
            last = b

        msg.poses = poses
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