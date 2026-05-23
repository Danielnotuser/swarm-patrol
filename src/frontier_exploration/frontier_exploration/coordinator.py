#!/usr/bin/env python3

import math
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple


@dataclass
class ReservationData:
    robot_id: int
    x: float
    y: float
    timestamp: float

    def is_expired(self, ttl: float) -> bool:
        return (time.time() - self.timestamp) > ttl


class Coordinator:
    def __init__(
        self,
        robot_id: int,
        robot_count: int,
        dispersion_threshold: float = 4.0,
        exclusion_radius: float = 3.0,
        reservation_ttl: float = 30.0,
        logger=None,
    ):
        self.robot_id = robot_id
        self.robot_count = robot_count
        self.dispersion_threshold = dispersion_threshold
        self.exclusion_radius = exclusion_radius
        self.reservation_ttl = reservation_ttl
        self.logger = logger

        self._reservations: Dict[int, ReservationData] = {}
        self._other_robot_poses: Dict[int, Tuple[float, float]] = {}

    def update_robot_poses(self, poses: Dict[int, Tuple[float, float]]) -> None:
        self._other_robot_poses = {
            rid: pos for rid, pos in poses.items()
            if rid != self.robot_id
        }

    def update_reservation(self, robot_id: int, x: float, y: float) -> None:
        if robot_id != self.robot_id:
            self._reservations[robot_id] = ReservationData(
                robot_id=robot_id,
                x=x,
                y=y,
                timestamp=time.time(),
            )

    def _clean_expired(self) -> None:
        expired = [rid for rid, res in self._reservations.items()
                   if res.is_expired(self.reservation_ttl)]
        for rid in expired:
            self._reservations.pop(rid, None)

    def select_best_frontiers(
        self,
        frontiers: List[Tuple[float, float, float, float]],
        my_pose: Tuple[float, float],
    ) -> List[Tuple[float, float, float]]:
        self._clean_expired()

        if not frontiers:
            return []

        dists = [f'{d:.2f}' for _, _, d, _ in frontiers]
        if self.logger:
            self.logger.info(f'[coordinator] Received {len(frontiers)} frontiers with dists: {dists[:10]}')

        near_dropped = 0
        filtered = []
        for fx, fy, dist, unknown_ratio in frontiers:
            if dist < 0.3:
                near_dropped += 1
                continue

            too_close = False

            for rid, pos in self._other_robot_poses.items():
                d = math.hypot(fx - pos[0], fy - pos[1])
                if d < self.exclusion_radius:
                    too_close = True
                    break

            if too_close:
                continue

            if not too_close:
                filtered.append((fx, fy, dist, unknown_ratio))

        if not filtered:
            if self.logger:
                self.logger.info(f'[coordinator] No frontiers after filtering: {near_dropped} near-dropped')
            return []

        if self.logger:
            self.logger.info(f'[coordinator] {len(filtered)} frontiers passed filters')
        scored = []
        for fx, fy, dist, unknown_ratio in filtered:
            penalty = 0.0

            for rid, pos in self._other_robot_poses.items():
                d = math.hypot(fx - pos[0], fy - pos[1])
                if d < self.dispersion_threshold:
                    penalty += (self.dispersion_threshold - d) * 2.0

            for rid, res in self._reservations.items():
                d = math.hypot(fx - res.x, fy - res.y)
                if d < self.dispersion_threshold:
                    penalty += 5.0

            final_score = dist + penalty
            scored.append((fx, fy, final_score))

        if not scored:
            return []

        scored.sort(key=lambda s: s[2])
        if self.logger:
            self.logger.info(f'[coordinator] Top-5 scored: ' + ', '.join(f'({x:.1f},{y:.1f}):{s:.1f}' for x, y, s in scored[:5]))
        return scored

    def set_my_reservation(self, x: float, y: float) -> None:
        self._reservations[self.robot_id] = ReservationData(
            robot_id=self.robot_id,
            x=x,
            y=y,
            timestamp=time.time(),
        )

    def clear_my_reservation(self) -> None:
        self._reservations.pop(self.robot_id, None)

    def clear_other_reservation(self, robot_id: int) -> None:
        self._reservations.pop(robot_id, None)