from __future__ import annotations

from collections import Counter
from dataclasses import dataclass, field
from time import perf_counter

import numpy as np

from uav_cooperation.environment.voxel_grid import VoxelGrid

from uav_cooperation.realtime_avoidance.local_avoidance import AvoidanceDecision, NeighborState, choose_safe_velocity
from uav_cooperation.realtime_avoidance.config import OnlineAvoidanceConfig
from uav_cooperation.realtime_avoidance.obstacles import SphericalObstacle, closest_approach, segment_sphere_distance
from uav_cooperation.scheduling.trajectory import TimedSegment, TimedTrajectory

Point3D = tuple[float, float, float]


@dataclass(frozen=True)
class OnlineScenario:
    name: str
    description: str
    obstacles: tuple[SphericalObstacle, ...] = ()


@dataclass
class AgentState:
    trajectory: TimedTrajectory
    position: np.ndarray
    segment_index: int = 0
    segment_elapsed_s: float = 0.0
    previous_velocity: np.ndarray = field(default_factory=lambda: np.zeros(3))
    completion_time_s: float | None = None
    visited_viewpoints: set[int] = field(default_factory=set)

    @property
    def vehicle_id(self) -> int:
        return self.trajectory.vehicle_id

    @property
    def completed(self) -> bool:
        return self.completion_time_s is not None

    @property
    def current_segment(self) -> TimedSegment | None:
        if self.segment_index >= len(self.trajectory.segments):
            return None
        return self.trajectory.segments[self.segment_index]

    def active_at(self, time_s: float) -> bool:
        return time_s >= self.trajectory.start_delay_s - 1e-9 and not self.completed


@dataclass(frozen=True)
class SimulationResult:
    scenario: OnlineScenario
    completed: bool
    completion_time_s: float
    airborne_durations_s: dict[int, float]
    coverage_count: int
    collision_count: int
    inter_uav_conflict_count: int
    minimum_inter_uav_separation_m: float
    minimum_obstacle_clearance_m: float
    mode_counts: dict[str, int]
    trigger_counts: dict[str, int]
    planning_cycle_count: int
    planning_time_p95_ms: float
    planning_time_max_ms: float
    local_astar_invocations: int
    trajectory_rows: tuple[dict[str, object], ...]
    planning_rows: tuple[dict[str, object], ...]
    obstacle_events: tuple[dict[str, object], ...]


def _longest_motion_segment(trajectory: TimedTrajectory) -> TimedSegment:
    segments = [
        segment
        for segment in trajectory.segments
        if segment.mode != "dwell" and segment.duration_s >= 5.0
    ]
    if not segments:
        raise RuntimeError("Trajectory has no usable motion segment")
    return max(segments, key=lambda segment: segment.duration_s)


def build_online_scenarios(
    trajectories: tuple[TimedTrajectory, ...],
) -> tuple[OnlineScenario, ...]:
    by_vehicle = {trajectory.vehicle_id: trajectory for trajectory in trajectories}
    usable = sorted(vehicle_id for vehicle_id, trajectory in by_vehicle.items()
                    if any(segment.mode != "dwell" and segment.duration_s >= 5.0
                           for segment in trajectory.segments))
    if not usable:
        return (OnlineScenario("baseline", "No additional obstacles"),)
    static_segment = _longest_motion_segment(by_vehicle[usable[0]])
    static_time = (static_segment.start_s + static_segment.end_s) / 2.0
    static_center = tuple(
        float(value) for value in static_segment.position_at(static_time)
    )
    static_obstacle = SphericalObstacle(
        obstacle_id="unexpected_static_sphere",
        kind="static",
        radius_m=8.0,
        center_at_start_m=static_center,
        active_start_s=max(0.0, static_time - 30.0),
    )

    moving_segment = _longest_motion_segment(by_vehicle[usable[-1]])
    crossing_time = (moving_segment.start_s + moving_segment.end_s) / 2.0
    crossing_point = moving_segment.position_at(crossing_time)
    flight_direction = moving_segment.velocity
    perpendicular = np.cross(flight_direction, np.asarray([0.0, 0.0, 1.0]))
    if np.linalg.norm(perpendicular) <= 1e-9:
        perpendicular = np.cross(flight_direction, np.asarray([1.0, 0.0, 0.0]))
    perpendicular = perpendicular / np.linalg.norm(perpendicular)
    obstacle_speed = 3.0
    lead_time = 20.0
    moving_velocity = obstacle_speed * perpendicular
    moving_start = crossing_point - moving_velocity * lead_time
    moving_obstacle = SphericalObstacle(
        obstacle_id="crossing_moving_sphere",
        kind="moving",
        radius_m=2.0,
        center_at_start_m=tuple(float(value) for value in moving_start),
        velocity_mps=tuple(float(value) for value in moving_velocity),
        active_start_s=max(0.0, crossing_time - lead_time),
        active_end_s=crossing_time + lead_time,
    )
    return (
        OnlineScenario(
            name="baseline",
            description="输入场景，无新增障碍；在线循环持续检查邻机",
        ),
        OnlineScenario(
            name="static_surprise",
            description="有效航段中途出现半径8 m静态球形障碍",
            obstacles=(static_obstacle,),
        ),
        OnlineScenario(
            name="moving_obstacle",
            description="移动球形障碍横穿有效计划航段",
            obstacles=(moving_obstacle,),
        ),
        OnlineScenario(
            name="combined_obstacles",
            description="输入场景，同时包含新增静态障碍和移动障碍",
            obstacles=(static_obstacle, moving_obstacle),
        ),
    )


def _obstacle_clearance_for_step(
    start: np.ndarray,
    end: np.ndarray,
    time_s: float,
    dt: float,
    obstacle: SphericalObstacle,
    uav_radius_m: float,
) -> float:
    if obstacle.kind == "static":
        center = obstacle.center_at(time_s)
        center_distance = segment_sphere_distance(start, end, center)
    else:
        velocity = (end - start) / dt
        center_distance, _ = closest_approach(
            start,
            velocity,
            obstacle.center_at(time_s),
            obstacle.velocity,
            dt,
        )
    return center_distance - obstacle.radius_m - uav_radius_m


def simulate_online_scenario(
    scenario: OnlineScenario,
    trajectories: tuple[TimedTrajectory, ...],
    grid: VoxelGrid,
    config: OnlineAvoidanceConfig,
    priority_order: tuple[int, ...] | None = None,
    task_nodes: set[int] | None = None,
) -> SimulationResult:
    dt = config.time_step_s
    agents = {
        trajectory.vehicle_id: AgentState(
            trajectory=trajectory,
            position=np.asarray(trajectory.segments[0].start, dtype=float),
        )
        for trajectory in trajectories
    }
    priority_order = priority_order or tuple(sorted(agents))
    if set(priority_order) != set(agents) or len(priority_order) != len(agents):
        raise ValueError("priority_order must contain each active vehicle exactly once")
    task_nodes = task_nodes if task_nodes is not None else set().union(
        *(set(trajectory.route[1:-1]) for trajectory in trajectories))
    max_time = max(trajectory.end_s for trajectory in trajectories) + (
        config.maximum_simulation_overtime_s
    )
    trajectory_rows: list[dict[str, object]] = []
    planning_rows: list[dict[str, object]] = []
    mode_counts: Counter[str] = Counter()
    trigger_counts: Counter[str] = Counter()
    collision_count = 0
    inter_uav_conflict_count = 0
    minimum_inter_uav = float("inf")
    minimum_obstacle_clearance = float("inf")
    local_astar_invocations = 0
    time_s = 0.0
    simulation_started = perf_counter()

    while time_s <= max_time + 1e-9:
        if all(agent.completed for agent in agents.values()):
            break
        old_positions = {
            vehicle_id: agent.position.copy() for vehicle_id, agent in agents.items()
        }
        chosen: dict[int, AvoidanceDecision] = {}
        cycle_velocities = {
            vehicle_id: agent.previous_velocity.copy()
            for vehicle_id, agent in agents.items()
        }

        for vehicle_id in priority_order:
            agent = agents[vehicle_id]
            segment = agent.current_segment
            if agent.completed or time_s < agent.trajectory.start_delay_s - 1e-9:
                continue
            if segment is None:
                agent.completion_time_s = time_s
                continue
            if segment.mode == "dwell":
                decision = AvoidanceDecision(
                    velocity_mps=(0.0, 0.0, 0.0),
                    mode="dwell",
                    trigger="none",
                    predicted_min_separation_m=float("inf"),
                    evaluated_candidates=0,
                    local_astar_expanded_nodes=0,
                    planning_time_ms=0.0,
                )
            else:
                target = np.asarray(segment.end, dtype=float)
                delta = target - agent.position
                distance = float(np.linalg.norm(delta))
                nominal_speed = float(np.linalg.norm(segment.velocity))
                command_speed = min(nominal_speed, distance / dt) if distance > 0 else 0.0
                preferred = command_speed * delta / distance if distance > 1e-12 else np.zeros(3)
                neighbors = tuple(
                    NeighborState(
                        vehicle_id=other_id,
                        position_m=tuple(float(value) for value in other.position),
                        velocity_mps=tuple(
                            float(value) for value in cycle_velocities[other_id]
                        ),
                        active=other.active_at(time_s),
                    )
                    for other_id, other in agents.items()
                    if other_id != vehicle_id
                )
                decision = choose_safe_velocity(
                    vehicle_id=vehicle_id,
                    time_s=time_s,
                    position_m=tuple(float(value) for value in agent.position),
                    target_m=tuple(float(value) for value in target),
                    preferred_velocity_mps=tuple(float(value) for value in preferred),
                    previous_velocity_mps=tuple(
                        float(value) for value in agent.previous_velocity
                    ),
                    obstacles=scenario.obstacles,
                    neighbors=neighbors,
                    base_point_is_free=grid.point_is_free,
                    base_segment_is_free=grid.line_is_free,
                    config=config,
                )
            chosen[vehicle_id] = decision
            cycle_velocities[vehicle_id] = np.asarray(decision.velocity_mps, dtype=float)
            mode_counts[decision.mode] += 1
            if decision.trigger != "none":
                trigger_counts[decision.trigger] += 1
            if decision.mode == "local_astar":
                local_astar_invocations += 1
            planning_rows.append(
                {
                    "scenario": scenario.name,
                    "vehicle_id": vehicle_id,
                    "time_s": time_s,
                    "mode": decision.mode,
                    "trigger": decision.trigger,
                    "safe_velocity_x_mps": decision.velocity_mps[0],
                    "safe_velocity_y_mps": decision.velocity_mps[1],
                    "safe_velocity_z_mps": decision.velocity_mps[2],
                    "predicted_min_separation_m": decision.predicted_min_separation_m,
                    "planning_time_ms": decision.planning_time_ms,
                    "evaluated_candidates": decision.evaluated_candidates,
                    "local_astar_expanded_nodes": decision.local_astar_expanded_nodes,
                }
            )

        active_during_step: dict[int, bool] = {}
        for vehicle_id, agent in agents.items():
            segment = agent.current_segment
            active = agent.active_at(time_s) and segment is not None
            active_during_step[vehicle_id] = active
            if not active:
                continue
            decision = chosen.get(vehicle_id)
            if segment.mode == "dwell":
                agent.previous_velocity = np.zeros(3)
                agent.segment_elapsed_s += dt
                if agent.segment_elapsed_s + 1e-9 >= segment.duration_s:
                    agent.segment_index += 1
                    agent.segment_elapsed_s = 0.0
            elif decision is not None:
                velocity = np.asarray(decision.velocity_mps, dtype=float)
                target = np.asarray(segment.end, dtype=float)
                next_position = agent.position + velocity * dt
                before_distance = float(np.linalg.norm(target - agent.position))
                after_distance = float(np.linalg.norm(target - next_position))
                if (
                    after_distance <= 1e-6
                ):
                    next_position = target
                agent.position = next_position
                agent.previous_velocity = velocity
                if np.linalg.norm(agent.position - target) <= 1e-6:
                    next_index = agent.segment_index + 1
                    if (next_index == len(agent.trajectory.segments)
                        or agent.trajectory.segments[next_index].route_leg_index != segment.route_leg_index
                        or agent.trajectory.segments[next_index].mode == "dwell"):
                        route_leg = segment.route_leg_index
                        viewpoint_node = agent.trajectory.route[route_leg + 1]
                        if viewpoint_node in task_nodes:
                            agent.visited_viewpoints.add(viewpoint_node)
                    agent.segment_index += 1
                    agent.segment_elapsed_s = 0.0
                if agent.segment_index >= len(agent.trajectory.segments):
                    agent.completion_time_s = time_s + dt

        for first_id in sorted(agents):
            for second_id in sorted(agents):
                if second_id <= first_id:
                    continue
                if not (active_during_step[first_id] and active_during_step[second_id]):
                    continue
                first = agents[first_id]
                second = agents[second_id]
                first_velocity = (first.position - old_positions[first_id]) / dt
                second_velocity = (second.position - old_positions[second_id]) / dt
                distance, _ = closest_approach(
                    old_positions[first_id],
                    first_velocity,
                    old_positions[second_id],
                    second_velocity,
                    dt,
                )
                minimum_inter_uav = min(minimum_inter_uav, distance)
                if distance < config.minimum_inter_uav_separation_m - 1e-8:
                    inter_uav_conflict_count += 1

        for vehicle_id, agent in agents.items():
            if not active_during_step[vehicle_id]:
                continue
            if not grid.line_is_free(tuple(old_positions[vehicle_id]), tuple(agent.position)):
                collision_count += 1
            for obstacle in scenario.obstacles:
                if not obstacle.is_active(time_s):
                    continue
                clearance = _obstacle_clearance_for_step(
                    old_positions[vehicle_id],
                    agent.position,
                    time_s,
                    dt,
                    obstacle,
                    config.uav_collision_radius_m,
                )
                minimum_obstacle_clearance = min(minimum_obstacle_clearance, clearance)
                if clearance < -1e-8:
                    collision_count += 1

        for vehicle_id, agent in agents.items():
            segment = agent.current_segment
            if agent.completed:
                mode = "complete"
            elif time_s < agent.trajectory.start_delay_s - 1e-9:
                mode = "preflight"
            elif segment is not None and segment.mode == "dwell":
                mode = "dwell"
            else:
                mode = chosen.get(vehicle_id).mode if vehicle_id in chosen else "inactive"
            trajectory_rows.append(
                {
                    "scenario": scenario.name,
                    "vehicle_id": vehicle_id,
                    "time_s": time_s + dt,
                    "x_m": float(agent.position[0]),
                    "y_m": float(agent.position[1]),
                    "z_m": float(agent.position[2]),
                    "mode": mode,
                    "visited_viewpoint_count": len(agent.visited_viewpoints),
                }
            )
        time_s += dt

    completed = all(agent.completed for agent in agents.values())
    completion_time = max(
        (agent.completion_time_s or max_time) for agent in agents.values()
    )
    planning_times = np.asarray(
        [float(row["planning_time_ms"]) for row in planning_rows], dtype=float
    )
    all_visited = set().union(*(agent.visited_viewpoints for agent in agents.values()))
    airborne = {
        vehicle_id: (agent.completion_time_s or max_time)
        - agent.trajectory.start_delay_s
        for vehicle_id, agent in agents.items()
    }
    obstacle_events = tuple(obstacle.to_dict() for obstacle in scenario.obstacles)
    _ = perf_counter() - simulation_started
    return SimulationResult(
        scenario=scenario,
        completed=completed,
        completion_time_s=float(completion_time),
        airborne_durations_s=airborne,
        coverage_count=len(all_visited),
        collision_count=collision_count,
        inter_uav_conflict_count=inter_uav_conflict_count,
        minimum_inter_uav_separation_m=(
            minimum_inter_uav if np.isfinite(minimum_inter_uav) else float("nan")
        ),
        minimum_obstacle_clearance_m=(
            minimum_obstacle_clearance
            if np.isfinite(minimum_obstacle_clearance)
            else float("nan")
        ),
        mode_counts=dict(mode_counts),
        trigger_counts=dict(trigger_counts),
        planning_cycle_count=len(planning_rows),
        planning_time_p95_ms=(
            float(np.percentile(planning_times, 95)) if planning_times.size else 0.0
        ),
        planning_time_max_ms=(float(np.max(planning_times)) if planning_times.size else 0.0),
        local_astar_invocations=local_astar_invocations,
        trajectory_rows=tuple(trajectory_rows),
        planning_rows=tuple(planning_rows),
        obstacle_events=obstacle_events,
    )
