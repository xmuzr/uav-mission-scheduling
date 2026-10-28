from __future__ import annotations

from dataclasses import dataclass
from heapq import heappop, heappush
from itertools import count, product
from math import sqrt

import numpy as np

from uav_cooperation.models.geometry import Point3D

from uav_cooperation.environment.voxel_grid import GridIndex, VoxelGrid
from uav_cooperation.path_planning.postprocessing.simplification import simplify_grid_path


@dataclass(frozen=True)
class PathPlan:
    points: tuple[Point3D, ...]
    length_m: float
    raw_grid_nodes: int
    simplified_nodes: int
    expanded_nodes: int
    method: str


class AStarPlanner:
    def __init__(
        self,
        grid: VoxelGrid,
        max_expansions: int = 2_000_000,
        heuristic_weight: float = 1.2,
    ) -> None:
        self.grid = grid
        self.max_expansions = int(max_expansions)
        if heuristic_weight < 1.0:
            raise ValueError("heuristic_weight must be at least 1.0")
        self.heuristic_weight = float(heuristic_weight)
        self.moves = tuple(
            ((dx, dy, dz), sqrt(dx * dx + dy * dy + dz * dz) * grid.resolution_m)
            for dx, dy, dz in product((-1, 0, 1), repeat=3)
            if (dx, dy, dz) != (0, 0, 0)
        )

    def _heuristic(self, current: GridIndex, goal: GridIndex) -> float:
        delta = np.asarray(goal, dtype=float) - np.asarray(current, dtype=float)
        return float(np.linalg.norm(delta)) * self.grid.resolution_m

    def _reconstruct(
        self, came_from: dict[GridIndex, GridIndex], current: GridIndex
    ) -> list[GridIndex]:
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path

    def _search(self, start: GridIndex, goal: GridIndex) -> tuple[list[GridIndex], int]:
        queue: list[tuple[float, float, int, GridIndex]] = []
        serial = count()
        start_h = self._heuristic(start, goal)
        heappush(
            queue,
            (self.heuristic_weight * start_h, start_h, next(serial), start),
        )
        came_from: dict[GridIndex, GridIndex] = {}
        g_score: dict[GridIndex, float] = {start: 0.0}
        closed: set[GridIndex] = set()
        expanded = 0

        while queue:
            _, _, _, current = heappop(queue)
            if current in closed:
                continue
            if current == goal:
                return self._reconstruct(came_from, current), expanded

            closed.add(current)
            expanded += 1
            if expanded > self.max_expansions:
                raise RuntimeError(
                    f"A* exceeded max_expansions={self.max_expansions} for {start}->{goal}"
                )

            current_cost = g_score[current]
            for delta, step_cost in self.moves:
                neighbor = tuple(current[axis] + delta[axis] for axis in range(3))
                if not self.grid.is_free_index(neighbor):
                    continue
                if not self.grid.diagonal_transition_is_free(current, delta):
                    continue
                tentative = current_cost + step_cost
                if tentative + 1e-12 >= g_score.get(neighbor, float("inf")):
                    continue
                came_from[neighbor] = current
                g_score[neighbor] = tentative
                heuristic = self._heuristic(neighbor, goal)
                priority = tentative + self.heuristic_weight * heuristic
                # For equal weighted-f values, prefer the node closer to the goal.
                heappush(queue, (priority, heuristic, next(serial), neighbor))

        raise RuntimeError(f"No collision-free path found for {start}->{goal}")

    def _simplify(self, path: list[GridIndex]) -> list[GridIndex]:
        return simplify_grid_path(self.grid, path)

    @staticmethod
    def _length(points: list[Point3D]) -> float:
        return float(
            sum(
                np.linalg.norm(np.asarray(end) - np.asarray(start))
                for start, end in zip(points, points[1:])
            )
        )

    def plan(self, start_point: Point3D, goal_point: Point3D) -> PathPlan:
        start = self.grid.point_to_index(start_point)
        goal = self.grid.point_to_index(goal_point)
        if not self.grid.is_free_index(start):
            raise ValueError(f"Start point is occupied: {start_point}")
        if not self.grid.is_free_index(goal):
            raise ValueError(f"Goal point is occupied: {goal_point}")

        if self.grid.line_is_free(start_point, goal_point):
            points = [start_point, goal_point]
            return PathPlan(
                points=tuple(points),
                length_m=self._length(points),
                raw_grid_nodes=2,
                simplified_nodes=2,
                expanded_nodes=0,
                method="direct_line_of_sight",
            )

        raw_path, expanded = self._search(start, goal)
        simplified_path = self._simplify(raw_path)
        points = [self.grid.index_to_point(index) for index in simplified_path]
        return PathPlan(
            points=tuple(points),
            length_m=self._length(points),
            raw_grid_nodes=len(raw_path),
            simplified_nodes=len(simplified_path),
            expanded_nodes=expanded,
            method="astar_26_then_line_of_sight",
        )
