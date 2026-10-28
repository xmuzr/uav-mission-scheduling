from __future__ import annotations

from dataclasses import dataclass
from heapq import heappop, heappush
from itertools import count, product
from math import cos, radians, sin, sqrt
from time import perf_counter
from typing import Callable, Iterable

import numpy as np

from uav_cooperation.realtime_avoidance.config import OnlineAvoidanceConfig
from uav_cooperation.realtime_avoidance.obstacles import SphericalObstacle, closest_approach, segment_sphere_distance

Point3D = tuple[float, float, float]
Index3D = tuple[int, int, int]


@dataclass(frozen=True)
class NeighborState:
    vehicle_id: int
    position_m: Point3D
    velocity_mps: Point3D
    active: bool = True


@dataclass(frozen=True)
class AvoidanceDecision:
    velocity_mps: Point3D
    mode: str
    trigger: str
    predicted_min_separation_m: float
    evaluated_candidates: int
    local_astar_expanded_nodes: int
    planning_time_ms: float


def _unit(vector: np.ndarray) -> np.ndarray:
    norm = float(np.linalg.norm(vector))
    if norm <= 1e-12:
        return np.zeros(3, dtype=float)
    return vector / norm


def _direction_basis(forward: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    forward = _unit(forward)
    reference = np.asarray([0.0, 0.0, 1.0])
    if abs(float(np.dot(forward, reference))) > 0.95:
        reference = np.asarray([1.0, 0.0, 0.0])
    side = _unit(np.cross(reference, forward))
    up = _unit(np.cross(forward, side))
    return forward, side, up


def _candidate_velocities(
    preferred_velocity: np.ndarray,
    config: OnlineAvoidanceConfig,
) -> Iterable[np.ndarray]:
    speed = float(np.linalg.norm(preferred_velocity))
    if speed <= 1e-12:
        yield np.zeros(3, dtype=float)
        return
    forward, side, up = _direction_basis(preferred_velocity)
    seen: set[tuple[float, float, float]] = set()
    for scale in config.candidate_speed_scales:
        if scale <= 1e-12:
            candidate = np.zeros(3, dtype=float)
            key = (0.0, 0.0, 0.0)
            if key not in seen:
                seen.add(key)
                yield candidate
            continue
        for elevation_degree in config.candidate_elevation_degrees:
            elevation = radians(elevation_degree)
            for azimuth_degree in config.candidate_azimuth_degrees:
                azimuth = radians(azimuth_degree)
                direction = (
                    cos(elevation) * cos(azimuth) * forward
                    + cos(elevation) * sin(azimuth) * side
                    + sin(elevation) * up
                )
                candidate = speed * scale * _unit(direction)
                key = tuple(float(round(value, 9)) for value in candidate)
                if key not in seen:
                    seen.add(key)
                    yield candidate


def _combined_point_is_free(
    point: np.ndarray,
    time_s: float,
    base_point_is_free: Callable[[Point3D], bool],
    obstacles: tuple[SphericalObstacle, ...],
    config: OnlineAvoidanceConfig,
) -> bool:
    point_tuple = tuple(float(value) for value in point)
    if not base_point_is_free(point_tuple):
        return False
    for obstacle in obstacles:
        if obstacle.kind != "static" or not obstacle.is_active(time_s):
            continue
        effective_radius = (
            obstacle.radius_m
            + config.uav_collision_radius_m
            + config.static_safety_margin_m
        )
        if np.linalg.norm(point - obstacle.center_at(time_s)) < effective_radius - 1e-9:
            return False
    return True


def local_astar_path(
    start_m: Point3D,
    goal_m: Point3D,
    resolution_m: float,
    radius_m: float,
    max_expansions: int,
    point_is_free: Callable[[Point3D], bool],
    segment_is_free: Callable[[Point3D, Point3D], bool],
) -> tuple[tuple[Point3D, ...], int]:
    """Sparse local 26-neighbor A* used only when local velocity search is blocked."""
    start = np.asarray(start_m, dtype=float)
    requested_goal = np.asarray(goal_m, dtype=float)
    delta = requested_goal - start
    distance = float(np.linalg.norm(delta))
    if distance <= 1e-9:
        return (start_m,), 0
    proxy = requested_goal
    if distance > radius_m:
        proxy = start + delta / distance * radius_m
    goal_index = tuple(np.rint((proxy - start) / resolution_m).astype(int).tolist())
    bound = max(1, int(np.ceil(radius_m / resolution_m)))

    def point(index: Index3D) -> Point3D:
        value = start + np.asarray(index, dtype=float) * resolution_m
        return tuple(float(component) for component in value)

    def in_bounds(index: Index3D) -> bool:
        return all(-bound <= value <= bound for value in index) and (
            np.linalg.norm(np.asarray(index, dtype=float) * resolution_m)
            <= radius_m + resolution_m
        )

    if not point_is_free(point((0, 0, 0))):
        raise RuntimeError("Local A* start is occupied")
    if not in_bounds(goal_index) or not point_is_free(point(goal_index)):
        candidates = []
        for radius in range(1, 4):
            for offset in product(range(-radius, radius + 1), repeat=3):
                candidate = tuple(goal_index[axis] + offset[axis] for axis in range(3))
                if in_bounds(candidate) and point_is_free(point(candidate)):
                    candidates.append(candidate)
            if candidates:
                break
        if not candidates:
            raise RuntimeError("Local A* cannot find a free proxy goal")
        goal_index = min(
            candidates,
            key=lambda index: np.linalg.norm(np.asarray(point(index)) - proxy),
        )

    moves = tuple(
        ((dx, dy, dz), sqrt(dx * dx + dy * dy + dz * dz) * resolution_m)
        for dx, dy, dz in product((-1, 0, 1), repeat=3)
        if (dx, dy, dz) != (0, 0, 0)
    )
    origin: Index3D = (0, 0, 0)
    queue: list[tuple[float, int, Index3D]] = []
    serial = count()
    heappush(queue, (float(np.linalg.norm(np.asarray(goal_index))), next(serial), origin))
    costs: dict[Index3D, float] = {origin: 0.0}
    parents: dict[Index3D, Index3D] = {}
    closed: set[Index3D] = set()
    expanded = 0

    while queue:
        _, _, current = heappop(queue)
        if current in closed:
            continue
        if current == goal_index:
            indices = [current]
            while current in parents:
                current = parents[current]
                indices.append(current)
            indices.reverse()
            return tuple(point(index) for index in indices), expanded
        closed.add(current)
        expanded += 1
        if expanded > max_expansions:
            raise RuntimeError("Local A* exceeded max expansions")
        for offset, step_cost in moves:
            neighbor = tuple(current[axis] + offset[axis] for axis in range(3))
            if not in_bounds(neighbor) or neighbor in closed:
                continue
            current_point = point(current)
            neighbor_point = point(neighbor)
            if not point_is_free(neighbor_point) or not segment_is_free(
                current_point, neighbor_point
            ):
                continue
            tentative = costs[current] + step_cost
            if tentative + 1e-12 >= costs.get(neighbor, float("inf")):
                continue
            costs[neighbor] = tentative
            parents[neighbor] = current
            heuristic = float(
                np.linalg.norm(np.asarray(neighbor, dtype=float) - np.asarray(goal_index))
            ) * resolution_m
            heappush(queue, (tentative + 1.2 * heuristic, next(serial), neighbor))
    raise RuntimeError("Local A* found no path")


def choose_safe_velocity(
    *,
    vehicle_id: int,
    time_s: float,
    position_m: Point3D,
    target_m: Point3D,
    preferred_velocity_mps: Point3D,
    previous_velocity_mps: Point3D,
    obstacles: tuple[SphericalObstacle, ...],
    neighbors: tuple[NeighborState, ...],
    base_point_is_free: Callable[[Point3D], bool],
    base_segment_is_free: Callable[[Point3D, Point3D], bool],
    config: OnlineAvoidanceConfig,
) -> AvoidanceDecision:
    started = perf_counter()
    position = np.asarray(position_m, dtype=float)
    target = np.asarray(target_m, dtype=float)
    preferred = np.asarray(preferred_velocity_mps, dtype=float)
    previous = np.asarray(previous_velocity_mps, dtype=float)
    speed = float(np.linalg.norm(preferred))
    target_distance = float(np.linalg.norm(target - position))
    probe_duration = config.prediction_horizon_s
    if speed > 1e-12:
        probe_duration = min(probe_duration, max(config.time_step_s, target_distance / speed))

    visible_obstacles = tuple(
        obstacle
        for obstacle in obstacles
        if obstacle.is_active(time_s)
        and np.linalg.norm(obstacle.center_at(time_s) - position)
        <= config.sensor_range_m + obstacle.radius_m
    )

    def evaluate(candidate: np.ndarray) -> tuple[bool, float, str]:
        next_position = position + candidate * config.time_step_s
        if not base_segment_is_free(
            tuple(float(value) for value in position),
            tuple(float(value) for value in next_position),
        ):
            return False, 0.0, "bridge"
        predicted_min = float("inf")
        trigger = "none"
        for obstacle in visible_obstacles:
            obstacle_center = obstacle.center_at(time_s)
            if obstacle.kind == "static":
                candidate_end = position + candidate * probe_duration
                separation = segment_sphere_distance(
                    position, candidate_end, obstacle_center
                )
                required = (
                    obstacle.radius_m
                    + config.uav_collision_radius_m
                    + config.static_safety_margin_m
                )
                closest = separation
            else:
                closest, _ = closest_approach(
                    position,
                    candidate,
                    obstacle_center,
                    obstacle.velocity,
                    probe_duration,
                )
                required = obstacle.radius_m + config.uav_collision_radius_m
            predicted_min = min(predicted_min, closest)
            if closest < required - 1e-9:
                return False, closest, obstacle.kind
            if closest < required + 5.0:
                trigger = obstacle.kind
        for neighbor in neighbors:
            if not neighbor.active or neighbor.vehicle_id == vehicle_id:
                continue
            closest, _ = closest_approach(
                position,
                candidate,
                np.asarray(neighbor.position_m, dtype=float),
                np.asarray(neighbor.velocity_mps, dtype=float),
                probe_duration,
            )
            predicted_min = min(predicted_min, closest)
            if closest < config.minimum_inter_uav_separation_m - 1e-9:
                return False, closest, "neighbor"
            if closest < config.minimum_inter_uav_separation_m + 5.0:
                trigger = "neighbor"
        return True, predicted_min, trigger

    preferred_safe, preferred_separation, preferred_trigger = evaluate(preferred)
    if preferred_safe:
        return AvoidanceDecision(
            velocity_mps=tuple(float(value) for value in preferred),
            mode="follow",
            trigger=preferred_trigger,
            predicted_min_separation_m=preferred_separation,
            evaluated_candidates=1,
            local_astar_expanded_nodes=0,
            planning_time_ms=(perf_counter() - started) * 1000.0,
        )

    forward = _unit(preferred)
    previous_direction = _unit(previous)
    candidates: list[tuple[float, np.ndarray, float, str]] = []
    evaluated = 0
    for candidate in _candidate_velocities(preferred, config):
        evaluated += 1
        safe, separation, trigger = evaluate(candidate)
        if not safe:
            continue
        candidate_speed = float(np.linalg.norm(candidate))
        direction = _unit(candidate)
        target_cost = 1.0 - float(np.dot(direction, forward)) if candidate_speed else 2.0
        previous_cost = (
            1.0 - float(np.dot(direction, previous_direction))
            if candidate_speed and np.linalg.norm(previous_direction) > 0
            else 1.0
        )
        speed_cost = 1.0 - candidate_speed / max(speed, 1e-12)
        score = (
            config.target_direction_weight * target_cost
            + config.previous_direction_weight * previous_cost
            + config.speed_loss_weight * speed_cost
        )
        candidates.append((score, candidate, separation, trigger))
    if candidates:
        _, velocity, separation, candidate_trigger = min(candidates, key=lambda item: item[0])
        return AvoidanceDecision(
            velocity_mps=tuple(float(value) for value in velocity),
            mode="vfh_vo_avoid",
            trigger=preferred_trigger if preferred_trigger != "none" else candidate_trigger,
            predicted_min_separation_m=separation,
            evaluated_candidates=evaluated,
            local_astar_expanded_nodes=0,
            planning_time_ms=(perf_counter() - started) * 1000.0,
        )

    active_static = tuple(
        obstacle
        for obstacle in visible_obstacles
        if obstacle.kind == "static"
    )
    point_free = lambda value: _combined_point_is_free(
        np.asarray(value, dtype=float), time_s, base_point_is_free, active_static, config
    )

    def segment_free(start: Point3D, end: Point3D) -> bool:
        if not base_segment_is_free(start, end):
            return False
        for obstacle in active_static:
            required = obstacle.radius_m + config.uav_collision_radius_m
            if segment_sphere_distance(
                np.asarray(start), np.asarray(end), obstacle.center_at(time_s)
            ) < required - 1e-9:
                return False
        return True

    expanded = 0
    try:
        path, expanded = local_astar_path(
            position_m,
            target_m,
            config.local_astar_resolution_m,
            config.local_astar_radius_m,
            config.local_astar_max_expansions,
            point_free,
            segment_free,
        )
        if len(path) >= 2 and speed > 1e-12:
            direction = _unit(np.asarray(path[1]) - position)
            candidate = speed * direction
            safe, separation, trigger = evaluate(candidate)
            if safe:
                return AvoidanceDecision(
                    velocity_mps=tuple(float(value) for value in candidate),
                    mode="local_astar",
                    trigger=preferred_trigger if preferred_trigger != "none" else trigger,
                    predicted_min_separation_m=separation,
                    evaluated_candidates=evaluated,
                    local_astar_expanded_nodes=expanded,
                    planning_time_ms=(perf_counter() - started) * 1000.0,
                )
    except RuntimeError:
        pass

    wait = np.zeros(3, dtype=float)
    wait_safe, separation, wait_trigger = evaluate(wait)
    return AvoidanceDecision(
        velocity_mps=(0.0, 0.0, 0.0),
        mode="wait" if wait_safe else "emergency_stop",
        trigger=preferred_trigger if preferred_trigger != "none" else wait_trigger,
        predicted_min_separation_m=separation,
        evaluated_candidates=evaluated,
        local_astar_expanded_nodes=expanded,
        planning_time_ms=(perf_counter() - started) * 1000.0,
    )
