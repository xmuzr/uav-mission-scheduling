from __future__ import annotations

from dataclasses import asdict, dataclass

import numpy as np

Point3D = tuple[float, float, float]


@dataclass(frozen=True)
class SphericalObstacle:
    obstacle_id: str
    kind: str
    radius_m: float
    center_at_start_m: Point3D
    velocity_mps: Point3D = (0.0, 0.0, 0.0)
    active_start_s: float = 0.0
    active_end_s: float = float("inf")

    def __post_init__(self) -> None:
        if self.radius_m <= 0:
            raise ValueError("Obstacle radius must be positive")
        if self.active_end_s <= self.active_start_s:
            raise ValueError("Obstacle active interval must be positive")

    def is_active(self, time_s: float) -> bool:
        return self.active_start_s <= time_s <= self.active_end_s

    def center_at(self, time_s: float) -> np.ndarray:
        elapsed = max(0.0, time_s - self.active_start_s)
        return np.asarray(self.center_at_start_m, dtype=float) + elapsed * np.asarray(
            self.velocity_mps, dtype=float
        )

    @property
    def velocity(self) -> np.ndarray:
        return np.asarray(self.velocity_mps, dtype=float)

    def to_dict(self) -> dict[str, object]:
        payload = asdict(self)
        if not np.isfinite(self.active_end_s):
            payload["active_end_s"] = None
        return payload


def closest_approach(
    first_position: np.ndarray,
    first_velocity: np.ndarray,
    second_position: np.ndarray,
    second_velocity: np.ndarray,
    horizon_s: float,
) -> tuple[float, float]:
    """Return minimum center distance and its time for two constant velocities."""
    relative_position = np.asarray(first_position, dtype=float) - np.asarray(
        second_position, dtype=float
    )
    relative_velocity = np.asarray(first_velocity, dtype=float) - np.asarray(
        second_velocity, dtype=float
    )
    denominator = float(np.dot(relative_velocity, relative_velocity))
    closest_time = 0.0
    if denominator > 1e-12:
        closest_time = float(
            np.clip(
                -np.dot(relative_position, relative_velocity) / denominator,
                0.0,
                max(0.0, horizon_s),
            )
        )
    separation = relative_position + closest_time * relative_velocity
    return float(np.linalg.norm(separation)), closest_time


def segment_sphere_distance(
    start: np.ndarray, end: np.ndarray, center: np.ndarray
) -> float:
    start = np.asarray(start, dtype=float)
    end = np.asarray(end, dtype=float)
    center = np.asarray(center, dtype=float)
    delta = end - start
    denominator = float(np.dot(delta, delta))
    ratio = 0.0
    if denominator > 1e-12:
        ratio = float(np.clip(np.dot(center - start, delta) / denominator, 0.0, 1.0))
    return float(np.linalg.norm(start + ratio * delta - center))
