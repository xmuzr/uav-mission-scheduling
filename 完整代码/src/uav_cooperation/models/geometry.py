from __future__ import annotations

from dataclasses import dataclass
from math import hypot
from typing import Literal, Protocol

Point3D = tuple[float, float, float]
Region = Literal["bridge_below", "bridge_exterior", "bridge_above"]


class Obstacle(Protocol):
    def contains(self, point: Point3D, margin: float = 0.0) -> bool: ...


@dataclass(frozen=True)
class Cuboid:
    center: Point3D
    size: Point3D
    label: str = "cuboid"

    def contains(self, point: Point3D, margin: float = 0.0) -> bool:
        return all(
            abs(value - center) <= extent / 2.0 + margin
            for value, center, extent in zip(point, self.center, self.size)
        )


@dataclass(frozen=True)
class CylinderZ:
    center: Point3D
    radius: float
    height: float
    label: str = "cylinder"

    def contains(self, point: Point3D, margin: float = 0.0) -> bool:
        x, y, z = point
        cx, cy, cz = self.center
        radial_inside = hypot(x - cx, y - cy) <= self.radius + margin
        vertical_inside = abs(z - cz) <= self.height / 2.0 + margin
        return radial_inside and vertical_inside


@dataclass(frozen=True)
class Viewpoint:
    id: int
    position: Point3D
    region: Region


@dataclass(frozen=True)
class PlanningConfig:
    map_min: Point3D = (0.0, 0.0, 0.0)
    map_max: Point3D = (3000.0, 1400.0, 700.0)
    grid_resolution_m: float = 10.0
    speed_mps: float = 2.0
    dwell_seconds: float = 2.0
    drone_radius_m: float = 0.5
    safety_margin_m: float = 0.0
    unit: str = "m"


@dataclass(frozen=True)
class BridgeScenario:
    name: str
    obstacles: tuple[Obstacle, ...]
    viewpoints: tuple[Viewpoint, ...]
    depot: Point3D
    uav_count: int
    config: PlanningConfig
