from __future__ import annotations

from dataclasses import dataclass
from time import perf_counter

import numpy as np
from ortools.constraint_solver import pywrapcp, routing_enums_pb2



@dataclass(frozen=True)
class RouteMetrics:
    vehicle_id: int
    route: tuple[int, ...]
    viewpoint_count: int
    distance_m: float
    cruise_distance_m: float
    inspection_distance_m: float
    cruise_time_s: float
    inspection_time_s: float
    flight_time_s: float
    dwell_time_s: float
    task_time_s: float
    airborne_time_minutes: float
    remaining_flight_minutes: float
    battery_feasible: bool


@dataclass(frozen=True)
class AllocationSolution:
    routes: tuple[RouteMetrics, ...]
    makespan_s: float
    total_distance_m: float
    total_task_time_s: float
    load_cv: float
    solver: str
    status: str
    runtime_s: float
    objective_value: int


def route_metrics(
    vehicle_id: int,
    route: tuple[int, ...],
    matrix_m: np.ndarray,
    cruise_speed_mps: float,
    inspection_speed_mps: float,
    dwell_seconds: float,
    max_flight_minutes: float,
    depot_nodes: set[int] | None = None,
) -> RouteMetrics:
    depot_nodes = depot_nodes or {0}
    cruise_distance = float(
        sum(
            matrix_m[start, end]
            for start, end in zip(route, route[1:])
            if start in depot_nodes or end in depot_nodes
        )
    )
    inspection_distance = float(
        sum(
            matrix_m[start, end]
            for start, end in zip(route, route[1:])
            if start not in depot_nodes and end not in depot_nodes
        )
    )
    distance = cruise_distance + inspection_distance
    viewpoint_count = len(route) - 2
    cruise_time = cruise_distance / cruise_speed_mps
    inspection_time = inspection_distance / inspection_speed_mps
    flight_time = cruise_time + inspection_time
    dwell_time = (sum(dwell_seconds.get(node, 0.0) for node in route[1:-1])
                  if isinstance(dwell_seconds, dict) else viewpoint_count * dwell_seconds)
    task_time = flight_time + dwell_time
    return RouteMetrics(
        vehicle_id=vehicle_id,
        route=route,
        viewpoint_count=viewpoint_count,
        distance_m=distance,
        cruise_distance_m=cruise_distance,
        inspection_distance_m=inspection_distance,
        cruise_time_s=cruise_time,
        inspection_time_s=inspection_time,
        flight_time_s=flight_time,
        dwell_time_s=dwell_time,
        task_time_s=task_time,
        airborne_time_minutes=task_time / 60.0,
        remaining_flight_minutes=max_flight_minutes - task_time / 60.0,
        battery_feasible=task_time <= max_flight_minutes * 60.0 + 1e-9,
    )


def summarize_routes(
    routes: tuple[RouteMetrics, ...],
    solver: str,
    status: str,
    runtime_s: float,
    objective_value: int,
) -> AllocationSolution:
    task_times = np.asarray([route.task_time_s for route in routes], dtype=float)
    return AllocationSolution(
        routes=routes,
        makespan_s=float(np.max(task_times)),
        total_distance_m=float(sum(route.distance_m for route in routes)),
        total_task_time_s=float(np.sum(task_times)),
        load_cv=float(np.std(task_times) / np.mean(task_times)) if np.mean(task_times) > 1e-12 else 0.0,
        solver=solver,
        status=status,
        runtime_s=runtime_s,
        objective_value=objective_value,
    )

