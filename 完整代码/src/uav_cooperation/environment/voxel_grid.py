from __future__ import annotations

from dataclasses import dataclass
from itertools import combinations
from math import ceil, floor

import numpy as np

from uav_cooperation.models.geometry import BridgeScenario, Cuboid, CylinderZ, Point3D
from .mesh import TriangleMesh
from .collision import segment_is_free

GridIndex = tuple[int, int, int]


@dataclass(frozen=True)
class GridSummary:
    shape: GridIndex
    node_count: int
    occupied_node_count: int
    free_node_count: int
    resolution_m: float
    clearance_m: float


class VoxelGrid:
    """Dense occupancy grid whose nodes lie on the configured metric lattice."""

    def __init__(
        self,
        scenario: BridgeScenario,
        resolution_m: float,
        clearance_m: float = 0.0,
        line_sample_step_m: float | None = None,
    ) -> None:
        if resolution_m <= 0:
            raise ValueError("resolution_m must be positive")
        if clearance_m < 0:
            raise ValueError("clearance_m cannot be negative")

        self.scenario = scenario
        self.resolution_m = float(resolution_m)
        self.clearance_m = float(clearance_m)
        self.map_min = np.asarray(scenario.config.map_min, dtype=float)
        self.map_max = np.asarray(scenario.config.map_max, dtype=float)
        spans = (self.map_max - self.map_min) / self.resolution_m
        rounded = np.rint(spans)
        if not np.allclose(spans, rounded, atol=1e-9):
            raise ValueError("Map bounds must be divisible by grid resolution")
        self.shape: GridIndex = tuple((rounded.astype(int) + 1).tolist())
        self.line_sample_step_m = float(
            line_sample_step_m or self.resolution_m / 2.0
        )
        self.occupied = np.zeros(self.shape, dtype=bool)
        self._voxelize_obstacles()

    def _axis_range(self, low: float, high: float, axis: int) -> range:
        lower = max(0, ceil((low - self.map_min[axis]) / self.resolution_m - 1e-12))
        upper = min(
            self.shape[axis] - 1,
            floor((high - self.map_min[axis]) / self.resolution_m + 1e-12),
        )
        if lower > upper:
            return range(0)
        return range(lower, upper + 1)

    def _voxelize_obstacles(self) -> None:
        for obstacle in self.scenario.obstacles:
            if isinstance(obstacle, Cuboid):
                center = np.asarray(obstacle.center, dtype=float)
                half = np.asarray(obstacle.size, dtype=float) / 2.0 + self.clearance_m
                ranges = [
                    self._axis_range(center[axis] - half[axis], center[axis] + half[axis], axis)
                    for axis in range(3)
                ]
                if all(ranges):
                    self.occupied[
                        ranges[0].start : ranges[0].stop,
                        ranges[1].start : ranges[1].stop,
                        ranges[2].start : ranges[2].stop,
                    ] = True
            elif isinstance(obstacle, CylinderZ):
                cx, cy, cz = obstacle.center
                radius = obstacle.radius + self.clearance_m
                half_height = obstacle.height / 2.0 + self.clearance_m
                x_range = self._axis_range(cx - radius, cx + radius, 0)
                y_range = self._axis_range(cy - radius, cy + radius, 1)
                z_range = self._axis_range(cz - half_height, cz + half_height, 2)
                for ix in x_range:
                    x = self.map_min[0] + ix * self.resolution_m
                    for iy in y_range:
                        y = self.map_min[1] + iy * self.resolution_m
                        if (x - cx) ** 2 + (y - cy) ** 2 <= radius**2 + 1e-9:
                            self.occupied[ix, iy, z_range.start : z_range.stop] = True
            elif isinstance(obstacle,TriangleMesh):
                # Reserve every cell potentially touched by the inflated mesh.
                # The half-diagonal allowance prevents thin surfaces disappearing
                # between grid nodes; line checks still use original triangles.
                margin=self.clearance_m+np.sqrt(3)*self.resolution_m/2
                for component in obstacle.components:
                    ranges=[self._axis_range(component.low[a]-margin,component.high[a]+margin,a) for a in range(3)]
                    if not all(ranges):continue
                    slab=max(1,4096//(len(ranges[1])*len(ranges[2])))
                    for first in range(ranges[0].start,ranges[0].stop,slab):
                        indices=np.stack(np.meshgrid(np.arange(first,min(first+slab,ranges[0].stop)),
                                                     np.asarray(ranges[1]),np.asarray(ranges[2]),indexing='ij'),axis=-1)
                        points=self.map_min+indices.reshape(-1,3)*self.resolution_m
                        mask=component.contains_many(points,margin)
                        selected=indices.reshape(-1,3)[mask]
                        self.occupied[tuple(selected.T)]=True
            else:
                raise TypeError(f"Unsupported obstacle type: {type(obstacle)!r}")

    def summary(self) -> GridSummary:
        occupied_count = int(np.count_nonzero(self.occupied))
        node_count = int(self.occupied.size)
        return GridSummary(
            shape=self.shape,
            node_count=node_count,
            occupied_node_count=occupied_count,
            free_node_count=node_count - occupied_count,
            resolution_m=self.resolution_m,
            clearance_m=self.clearance_m,
        )

    def in_bounds_index(self, index: GridIndex) -> bool:
        return all(0 <= value < extent for value, extent in zip(index, self.shape))

    def is_free_index(self, index: GridIndex) -> bool:
        return self.in_bounds_index(index) and not bool(self.occupied[index])

    def point_to_index(self, point: Point3D) -> GridIndex:
        values = (np.asarray(point, dtype=float) - self.map_min) / self.resolution_m
        rounded = np.rint(values)
        if not np.allclose(values, rounded, atol=1e-8):
            raise ValueError(f"Point {point} is not aligned to the {self.resolution_m:g} m grid")
        index = tuple(rounded.astype(int).tolist())
        if not self.in_bounds_index(index):
            raise ValueError(f"Point {point} lies outside the grid")
        return index

    def index_to_point(self, index: GridIndex) -> Point3D:
        if not self.in_bounds_index(index):
            raise ValueError(f"Grid index {index} lies outside the grid")
        point = self.map_min + np.asarray(index, dtype=float) * self.resolution_m
        return tuple(point.tolist())

    def point_is_free(self, point: Point3D) -> bool:
        values = np.asarray(point, dtype=float)
        if np.any(values < self.map_min - 1e-9) or np.any(values > self.map_max + 1e-9):
            return False
        return not any(
            obstacle.contains(point, margin=self.clearance_m)
            for obstacle in self.scenario.obstacles
        )

    def line_is_free(self, start: Point3D, end: Point3D) -> bool:
        return segment_is_free(self.scenario,start,end,self.clearance_m,self.line_sample_step_m)

    def diagonal_transition_is_free(self, current: GridIndex, delta: GridIndex) -> bool:
        active_axes = [axis for axis, value in enumerate(delta) if value != 0]
        if len(active_axes) <= 1:
            return True
        for subset_size in range(1, len(active_axes)):
            for axes in combinations(active_axes, subset_size):
                intermediate = list(current)
                for axis in axes:
                    intermediate[axis] += delta[axis]
                if not self.is_free_index(tuple(intermediate)):
                    return False
        return True
