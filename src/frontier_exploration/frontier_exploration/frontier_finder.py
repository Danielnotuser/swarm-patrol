#!/usr/bin/env python3

from dataclasses import dataclass
from typing import List, Tuple

import math
import cv2
import numpy as np
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from cslam.utils.point_cloud2 import read_points, read_points_numpy_filtered


@dataclass
class Frontier:
    x: float
    y: float
    size: int
    dist_to_robot: float
    unknown_ratio: float


def quat_to_rot_matrix(q) -> np.ndarray:
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


class FrontierFinder:
    def __init__(
        self,
        resolution: float = 0.1,
        grid_size: float = 40.0,
        z_min: float = 0.0,
        cluster_dist: float = 1.5,
        min_passage_width: float = 1.0,
        origin_x: float = 0.0,
        origin_y: float = 0.0,
        logger=None,
    ):
        self.resolution = resolution
        self.grid_size = grid_size
        self.z_min = z_min
        self.cluster_dist = cluster_dist
        self.min_passage_width = min_passage_width
        self.logger = logger

        self._cols = int(grid_size / resolution)
        self._rows = int(grid_size / resolution)
        self._half = grid_size / 2.0
        self._origin_x = origin_x
        self._origin_y = origin_y

        self._persistent_grid = np.full((self._rows, self._cols), -1, dtype=np.int8)

    def _world_to_grid(self, wx: float, wy: float) -> Tuple[int, int]:
        c = int((wx - self._origin_x + self._half) / self.resolution)
        r = int((wy - self._origin_y + self._half) / self.resolution)
        return r, c

    @staticmethod
    def _bresenham(r0: int, c0: int, r1: int, c1: int) -> List[Tuple[int, int]]:
        cells = []
        dr = abs(r1 - r0)
        dc = -abs(c1 - c0)
        sr = 1 if r1 > r0 else -1
        sc = 1 if c1 > c0 else -1
        err = dr + dc
        r, c = r0, c0
        while True:
            cells.append((r, c))
            if r == r1 and c == c1:
                break
            e2 = 2 * err
            if e2 >= dc:
                err += dc
                r += sr
            if e2 <= dr:
                err += dr
                c += sc
        return cells[1:]

    def _transform_points(
        self,
        points: np.ndarray,
        tf_buffer,
        target_frame: str,
        source_frame: str,
    ) -> np.ndarray:
        try:
            from rclpy.time import Time
            tf = tf_buffer.lookup_transform(target_frame, source_frame, Time(seconds=0))
            R = quat_to_rot_matrix(tf.transform.rotation)
            t = tf.transform.translation

            if self.logger:
                self.logger.info(f'[frontier_finder] TF: t=({t.x:.3f}, {t.y:.3f}, {t.z:.3f})')

            T = np.eye(4, dtype=np.float64)
            T[:3, :3] = R
            T[0, 3] = float(t.x)
            T[1, 3] = float(t.y)
            T[2, 3] = float(t.z)

            ones = np.ones((points.shape[0], 1), dtype=np.float64)
            homogeneous = np.hstack([points, ones])
            transformed = (T @ homogeneous.T).T
            return transformed[:, :3]

        except Exception as e:
            if self.logger:
                self.logger.warn(f'[frontier_finder] Transform failed: {e}')
            return np.zeros((0, 3), dtype=np.float64)

    def _pointcloud_to_numpy(self, cloud: PointCloud2) -> Tuple[np.ndarray, np.ndarray]:
        if self.logger:
            self.logger.info(f'[frontier_finder] Cloud: width={cloud.width}, height={cloud.height}, '
                             f'point_step={cloud.point_step}, row_step={cloud.row_step}, '
                             f'data_len={len(cloud.data)}')

        points = read_points_numpy_filtered(cloud, skip_nans=True)

        if points.shape[0] == 0:
            return np.zeros((0, 3), dtype=np.float64), np.zeros((0, 3), dtype=np.float64)

        # Filter out inf/nan values first
        finite_mask = np.isfinite(points).all(axis=1)
        finite_points = points[finite_mask]

        if finite_points.shape[0] == 0:
            if self.logger:
                self.logger.warn('All points in cloud are inf/nan')
            return np.zeros((0, 3), dtype=np.float64), np.zeros((0, 3), dtype=np.float64)

        # Split into obstacles and floor
        obstacle_mask = finite_points[:, 2] >= self.z_min
        obstacles = finite_points[obstacle_mask]
        floor = finite_points[~obstacle_mask]

        if self.logger:
            self.logger.info(f'[frontier_finder] Points - Obstacles: {len(obstacles)}, Floor: {len(floor)}')
            if len(obstacles) > 0:
                self.logger.info(f'[frontier_finder]   Obstacles z range: [{obstacles[:,2].min():.3f}, {obstacles[:,2].max():.3f}]')
            if len(floor) > 0:
                self.logger.info(f'[frontier_finder]   Floor z range: [{floor[:,2].min():.3f}, {floor[:,2].max():.3f}]')

        return obstacles, floor

    def update_grid(self, obstacles: np.ndarray, floor: np.ndarray, robot_x: float, robot_y: float, max_range: float = 3.0) -> None:
        rr, rc = self._world_to_grid(robot_x, robot_y)

        for p in floor:
            dist = math.hypot(p[0] - robot_x, p[1] - robot_y)
            if dist > max_range:
                scale = max_range / dist
                end_x = robot_x + (p[0] - robot_x) * scale
                end_y = robot_y + (p[1] - robot_y) * scale
            else:
                end_x, end_y = p[0], p[1]
            r, c = self._world_to_grid(end_x, end_y)
            if 0 <= r < self._rows and 0 <= c < self._cols and self._persistent_grid[r, c] != 100:
                self._persistent_grid[r, c] = 0

        for p in obstacles:
            dist = math.hypot(p[0] - robot_x, p[1] - robot_y)
            if dist > max_range:
                scale = max_range / dist
                end_x = robot_x + (p[0] - robot_x) * scale
                end_y = robot_y + (p[1] - robot_y) * scale
            else:
                end_x, end_y = p[0], p[1]
            r, c = self._world_to_grid(end_x, end_y)
            if not (0 <= r < self._rows and 0 <= c < self._cols):
                continue
            for br, bc in self._bresenham(rr, rc, r, c):
                if 0 <= br < self._rows and 0 <= bc < self._cols and self._persistent_grid[br, bc] != 100:
                    self._persistent_grid[br, bc] = 0
            if dist <= max_range:
                self._persistent_grid[r, c] = 100

    def _find_frontier_cells(self, grid: np.ndarray) -> np.ndarray:
        frontier_mask = np.zeros_like(grid, dtype=np.uint8)

        for r in range(1, self._rows - 1):
            for c in range(1, self._cols - 1):
                # Frontier is a FREE cell (0) adjacent to UNKNOWN (-1)
                if grid[r, c] != 0:
                    continue
                
                has_unknown_neighbor = False
                for dr in (-1, 0, 1):
                    for dc in (-1, 0, 1):
                        if dr == 0 and dc == 0:
                            continue
                        nr, nc = r + dr, c + dc
                        if 0 <= nr < self._rows and 0 <= nc < self._cols:
                            if grid[nr, nc] == -1:
                                has_unknown_neighbor = True
                                break
                    if has_unknown_neighbor:
                        break
                if has_unknown_neighbor:
                    frontier_mask[r, c] = 1

        return frontier_mask

    def _cluster_frontiers(
        self, frontier_mask: np.ndarray, robot_x: float, robot_y: float,
    ) -> List[Frontier]:
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(
            frontier_mask.astype(np.uint8), connectivity=4
        )
        if self.logger:
            areas = [int(stats[l, cv2.CC_STAT_AREA]) for l in range(1, num_labels)]
            large = [a for a in areas if a >= 10]
            self.logger.info(
                f'[frontier_finder] connectedComponents: total={num_labels - 1}, '
                f'areas={areas[:20]}{"..." if len(areas) > 20 else ""}, '
                f'area>=10: {len(large)}'
            )

        frontiers = []
        for label in range(1, num_labels):
            area = int(stats[label, cv2.CC_STAT_AREA])
            if area < 10:
                continue

            mask = labels == label
            rows, cols = np.where(mask)
            wxs = self._origin_x - self._half + cols * self.resolution
            wys = self._origin_y - self._half + rows * self.resolution
            dists = np.sqrt((wxs - robot_x) ** 2 + (wys - robot_y) ** 2)
            dist = float(dists.mean())

            margin = 6
            sorted_idx = np.argsort(dists)[::-1]
            chosen_idx = sorted_idx[0]
            no_clear_candidate = True
            for idx in sorted_idx:
                r, c = rows[idx], cols[idx]
                clear = True
                for dr in range(-margin, margin + 1):
                    nr = r + dr
                    if nr < 0 or nr >= self._rows:
                        continue
                    for dc in range(-margin, margin + 1):
                        nc = c + dc
                        if nc < 0 or nc >= self._cols:
                            continue
                        if self._persistent_grid[nr, nc] == 100:
                            clear = False
                            break
                    if not clear:
                        break
                if clear:
                    chosen_idx = idx
                    no_clear_candidate = False
                    break

            if no_clear_candidate:
                if self.logger:
                    self.logger.info(
                        f'[frontier_finder]   label {label}: area={area}, dist={dist:.1f}, '
                        f'ALL cells blocked by obstacle (margin={margin})'
                    )
                continue

            wx = wxs[chosen_idx]
            wy = wys[chosen_idx]

            dx = wx - robot_x
            dy = wy - robot_y
            goal_dist = math.hypot(dx, dy)
            max_goal_dist = 5.0
            if goal_dist > max_goal_dist:
                scale = max_goal_dist / goal_dist
                wx = robot_x + dx * scale
                wy = robot_y + dy * scale
                cc = int((wx - (self._origin_x - self._half)) / self.resolution)
                rr = int((wy - (self._origin_y - self._half)) / self.resolution)
                clear = True
                for dr in range(-margin, margin + 1):
                    nr = rr + dr
                    if nr < 0 or nr >= self._rows:
                        continue
                    for dc in range(-margin, margin + 1):
                        nc = cc + dc
                        if nc < 0 or nc >= self._cols:
                            continue
                        if self._persistent_grid[nr, nc] == 100:
                            clear = False
                            break
                    if not clear:
                        break
                if not clear:
                    if self.logger:
                        self.logger.info(
                            f'[frontier_finder]   label {label}: area={area}, dist={dist:.1f}, '
                            f'clamped goal blocked by obstacle (margin={margin})'
                        )
                    continue

            cx, cy = centroids[label]
            r_min = max(0, int(cx) - 5)
            r_max = min(self._rows, int(cx) + 6)
            c_min = max(0, int(cy) - 5)
            c_max = min(self._cols, int(cy) + 6)
            local = frontier_mask[r_min:r_max, c_min:c_max]
            unknown_ratio = float(np.sum(local > 0)) / max(1, local.size)

            frontiers.append(
                Frontier(
                    x=wx,
                    y=wy,
                    size=area,
                    dist_to_robot=dist,
                    unknown_ratio=unknown_ratio,
                )
            )

        return frontiers

    def _passability_check(
        self, frontiers: List[Frontier], grid: np.ndarray
    ) -> List[Frontier]:
        passable = []

        for f in frontiers:
            r, c = self._world_to_grid(f.x, f.y)
            found = False
            for dr, dc in ((0, 0), (-1, 0), (1, 0), (0, -1), (0, 1)):
                nr, nc = r + dr, c + dc
                if 0 <= nr < self._rows and 0 <= nc < self._cols and grid[nr, nc] == 0:
                    found = True
                    break

            if found:
                passable.append(f)

        return passable

    def find_frontiers(
        self,
        cloud: PointCloud2,
        robot_x: float,
        robot_y: float,
        tf_buffer,
        target_frame: str,
        source_frame: str,
    ) -> Tuple[List[Frontier], np.ndarray]:
        obstacles, floor = self._pointcloud_to_numpy(cloud)
        
        if obstacles.shape[0] == 0 and floor.shape[0] == 0:
            if self.logger:
                self.logger.info('[frontier_finder] No valid points (obstacle or floor) found in cloud')
            return [], self._persistent_grid.copy()

        # Transform obstacles and floor
        points_to_transform = np.vstack([obstacles, floor]) if floor.shape[0] > 0 else obstacles
        
        points = self._transform_points(points_to_transform, tf_buffer, target_frame, source_frame)
        if self.logger and points.shape[0] > 0:
            self.logger.info(f'[frontier_finder] Points after transform: {points.shape[0]}')
            self.logger.info(f'[frontier_finder]   x range: [{points[:,0].min():.2f}, {points[:,0].max():.2f}]')
            self.logger.info(f'[frontier_finder]   y range: [{points[:,1].min():.2f}, {points[:,1].max():.2f}]')
            self.logger.info(f'[frontier_finder]   z range: [{points[:,2].min():.2f}, {points[:,2].max():.2f}]')
            self.logger.info(f'[frontier_finder]   robot at: ({robot_x:.2f}, {robot_y:.2f})')

        if points.shape[0] < 10:
            if self.logger:
                self.logger.info('[frontier_finder] Transform failed or returned too few points')
            return [], self._persistent_grid.copy()

        # Update persistent grid with new observations
        num_obstacles = obstacles.shape[0]
        self.update_grid(points[:num_obstacles], points[num_obstacles:], robot_x, robot_y)

        grid = self._persistent_grid
        occupied_cells = (grid == 100).sum()
        known_cells = (grid != -1).sum()
        if self.logger:
            self.logger.info(f'[frontier_finder] Persistent grid: {self._rows}x{self._cols}, known={known_cells}, occupied={occupied_cells}')

        frontier_mask = self._find_frontier_cells(grid)
        frontier_cells = frontier_mask.sum()
        if self.logger:
            self.logger.info(f'[frontier_finder] Frontier cells: {frontier_cells}')

        frontiers = self._cluster_frontiers(frontier_mask, robot_x, robot_y)
        if self.logger:
            self.logger.info(f'[frontier_finder] Clusters before passability: {len(frontiers)}')

        if not frontiers:
            if self.logger:
                self.logger.info('[frontier_finder] No frontier clusters found')
            return [], grid.copy()

        passable = self._passability_check(frontiers, grid)
        if self.logger:
            self.logger.info(f'[frontier_finder] Passable frontiers: {len(passable)}')

        return passable, grid.copy()