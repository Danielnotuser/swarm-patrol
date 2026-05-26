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


class Coordinator:
    def __init__(
        self,
        robot_id: int,
        robot_count: int,
        dispersion_threshold: float = 4.0,
        exclusion_radius: float = 3.0,
        logger=None,
    ):
        self.robot_id = robot_id
        self.robot_count = robot_count
        self.dispersion_threshold = dispersion_threshold
        self.exclusion_radius = exclusion_radius
        self.logger = logger

        self._reservations: Dict[int, ReservationData] = {}

    def update_reservation(self, robot_id: int, x: float, y: float) -> None:
        if robot_id != self.robot_id:
            self._reservations[robot_id] = ReservationData(
                robot_id=robot_id,
                x=x,
                y=y,
                timestamp=time.time(),
            )

    def select_best_frontiers(
        self,
        frontiers: List[Tuple[float, float, float, float]],
        my_pose: Tuple[float, float],
    ) -> List[Tuple[float, float, float]]:

        if not frontiers:
            return []

        dists = [f'{d:.2f}' for _, _, d, _ in frontiers]
        if self.logger:
            self.logger.info(f'[coordinator] Received {len(frontiers)} frontiers with dists: {dists[:10]}')

        near_dropped = 0
        filtered = []
        for fx, fy, dist, unknown_ratio in frontiers:
            if dist < 1.0:
                near_dropped += 1
                continue

            too_close = False

            for rid, res in self._reservations.items():
                if rid == self.robot_id:
                    continue
                d = math.hypot(fx - res.x, fy - res.y)
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
            for rid, res in self._reservations.items():
                if rid == self.robot_id:
                    continue
                d = math.hypot(fx - res.x, fy - res.y)
                penalty += (1 / d) * 20.0

                self.logger.info(f'[coordinator] {rid} reserv: cur penalty = {penalty}, d = {d}')

            final_score = dist * 0.01 + penalty
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
