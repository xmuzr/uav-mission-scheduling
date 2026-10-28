from __future__ import annotations

from dataclasses import asdict, dataclass

import numpy as np

from uav_cooperation.scheduling.trajectory import TimedSegment, TimedTrajectory


@dataclass(frozen=True)
class Conflict:
    vehicle_a: int
    vehicle_b: int
    interval_start_s: float
    interval_end_s: float
    closest_time_s: float
    minimum_distance_m: float
    position_a_m: tuple[float, float, float]
    position_b_m: tuple[float, float, float]
    mode_a: str
    mode_b: str

    def to_dict(self) -> dict[str, object]:
        return asdict(self)


def _segment_conflict(
    first: TimedSegment,
    second: TimedSegment,
    minimum_separation_m: float,
) -> Conflict | None:
    overlap_start = max(first.start_s, second.start_s)
    overlap_end = min(first.end_s, second.end_s)
    if overlap_end - overlap_start <= 1e-9:
        return None
    position_first = first.position_at(overlap_start)
    position_second = second.position_at(overlap_start)
    relative_position = position_first - position_second
    relative_velocity = first.velocity - second.velocity
    denominator = float(np.dot(relative_velocity, relative_velocity))
    offset = 0.0
    if denominator > 1e-12:
        offset = float(-np.dot(relative_position, relative_velocity) / denominator)
        offset = min(overlap_end - overlap_start, max(0.0, offset))
    closest_time = overlap_start + offset
    closest_first = first.position_at(closest_time)
    closest_second = second.position_at(closest_time)
    distance = float(np.linalg.norm(closest_first - closest_second))
    if distance + 1e-9 >= minimum_separation_m:
        return None
    return Conflict(
        vehicle_a=min(first.vehicle_id, second.vehicle_id),
        vehicle_b=max(first.vehicle_id, second.vehicle_id),
        interval_start_s=overlap_start,
        interval_end_s=overlap_end,
        closest_time_s=closest_time,
        minimum_distance_m=distance,
        position_a_m=tuple(float(value) for value in closest_first),
        position_b_m=tuple(float(value) for value in closest_second),
        mode_a=first.mode,
        mode_b=second.mode,
    )


def detect_pair_conflicts(
    first: TimedTrajectory,
    second: TimedTrajectory,
    minimum_separation_m: float,
    stop_after_first: bool = False,
) -> list[Conflict]:
    conflicts: list[Conflict] = []
    for segment_first in first.segments:
        for segment_second in second.segments:
            conflict = _segment_conflict(
                segment_first, segment_second, minimum_separation_m
            )
            if conflict is not None:
                conflicts.append(conflict)
                if stop_after_first:
                    return conflicts
    return conflicts


def detect_conflicts(
    trajectories: tuple[TimedTrajectory, ...],
    minimum_separation_m: float,
    stop_after_first: bool = False,
) -> list[Conflict]:
    conflicts: list[Conflict] = []
    for first_index in range(len(trajectories)):
        for second_index in range(first_index + 1, len(trajectories)):
            pair_conflicts = detect_pair_conflicts(
                trajectories[first_index],
                trajectories[second_index],
                minimum_separation_m,
                stop_after_first=stop_after_first,
            )
            conflicts.extend(pair_conflicts)
            if conflicts and stop_after_first:
                return conflicts
    return conflicts

