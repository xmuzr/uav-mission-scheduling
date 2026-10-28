from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class OnlineAvoidanceConfig:
    planning_update_hz: float = 2.0
    prediction_horizon_s: float = 10.0
    sensor_range_m: float = 100.0
    uav_collision_radius_m: float = 0.5
    static_safety_margin_m: float = 0.0
    minimum_inter_uav_separation_m: float = 20.0
    candidate_azimuth_degrees: tuple[float, ...] = (
        0.0, -15.0, 15.0, -30.0, 30.0, -45.0, 45.0,
        -60.0, 60.0, -90.0, 90.0, 180.0,
    )
    candidate_elevation_degrees: tuple[float, ...] = (
        0.0, 15.0, -15.0, 30.0, -30.0, 45.0, -45.0, 60.0, -60.0,
    )
    candidate_speed_scales: tuple[float, ...] = (1.0, 0.75, 0.5, 0.0)
    target_direction_weight: float = 5.0
    previous_direction_weight: float = 1.0
    speed_loss_weight: float = 0.5
    local_astar_resolution_m: float = 5.0
    local_astar_radius_m: float = 80.0
    local_astar_max_expansions: int = 100_000
    maximum_simulation_overtime_s: float = 600.0
    scenario_seed: int = 20_260_911
    realtime_definition: str = (
        "planning_cycle_p95_below_cycle_period_not_hard_realtime"
    )

    @property
    def time_step_s(self) -> float:
        return 1.0 / self.planning_update_hz

    @classmethod
    def from_json(cls, path: Path) -> "OnlineAvoidanceConfig":
        data = json.loads(path.read_text(encoding="utf-8"))
        for key in (
            "candidate_azimuth_degrees",
            "candidate_elevation_degrees",
            "candidate_speed_scales",
        ):
            data[key] = tuple(float(value) for value in data[key])
        config = cls(**data)
        positive = (
            config.planning_update_hz,
            config.prediction_horizon_s,
            config.sensor_range_m,
            config.uav_collision_radius_m,
            config.minimum_inter_uav_separation_m,
            config.local_astar_resolution_m,
            config.local_astar_radius_m,
            config.maximum_simulation_overtime_s,
        )
        if min(positive) <= 0:
            raise ValueError("Online avoidance positive parameters must be greater than zero")
        if config.static_safety_margin_m < 0:
            raise ValueError("static_safety_margin_m cannot be negative")
        if config.minimum_inter_uav_separation_m < 2 * config.uav_collision_radius_m:
            raise ValueError("Inter-UAV separation cannot be smaller than physical diameter")
        if config.local_astar_max_expansions < 1:
            raise ValueError("local_astar_max_expansions must be positive")
        return config
