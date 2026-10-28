from __future__ import annotations

from dataclasses import dataclass, replace

import numpy as np

from .path_data import PathData as Stage3Data

Point3D = tuple[float, float, float]


@dataclass(frozen=True)
class TimedSegment:
    vehicle_id: int
    start_s: float
    end_s: float
    start: Point3D
    end: Point3D
    mode: str
    route_leg_index: int

    @property
    def duration_s(self) -> float:
        return self.end_s - self.start_s

    def position_at(self, time_s: float) -> np.ndarray:
        if self.duration_s <= 1e-12:
            return np.asarray(self.end, dtype=float)
        ratio = min(1.0, max(0.0, (time_s - self.start_s) / self.duration_s))
        return np.asarray(self.start, dtype=float) + ratio * (
            np.asarray(self.end, dtype=float) - np.asarray(self.start, dtype=float)
        )

    @property
    def velocity(self) -> np.ndarray:
        if self.duration_s <= 1e-12:
            return np.zeros(3, dtype=float)
        return (np.asarray(self.end) - np.asarray(self.start)) / self.duration_s


@dataclass(frozen=True)
class TimedTrajectory:
    vehicle_id: int
    route: tuple[int, ...]
    route_labels: tuple[str, ...]
    start_delay_s: float
    airborne_duration_s: float
    segments: tuple[TimedSegment, ...]

    @property
    def start_s(self) -> float:
        return self.start_delay_s

    @property
    def end_s(self) -> float:
        return self.start_delay_s + self.airborne_duration_s

    def shifted(self, delay_s: float) -> "TimedTrajectory":
        if delay_s < 0:
            raise ValueError("Departure delay cannot be negative")
        shift = delay_s - self.start_delay_s
        return replace(
            self,
            start_delay_s=delay_s,
            segments=tuple(
                replace(segment, start_s=segment.start_s + shift, end_s=segment.end_s + shift)
                for segment in self.segments
            ),
        )

    def state_at(self, time_s: float) -> tuple[Point3D, str]:
        if time_s < self.start_s - 1e-9 or time_s > self.end_s + 1e-9:
            raise ValueError("Requested time is outside the active trajectory")
        for segment in self.segments:
            if segment.start_s - 1e-9 <= time_s <= segment.end_s + 1e-9:
                return tuple(float(value) for value in segment.position_at(time_s)), segment.mode
        return self.segments[-1].end, self.segments[-1].mode


def build_timed_trajectory(
    data: Stage3Data,
    route_payload: dict[str, object],
    depot_nodes: set[int],
    cruise_speed_mps: float,
    inspection_speed_mps: float,
    dwell_seconds: float | dict[int, float],
    start_delay_s: float = 0.0,
) -> TimedTrajectory:
    vehicle_id = int(route_payload["vehicle_id"])
    route = tuple(int(node) for node in route_payload["route"])
    labels = tuple(data.labels[node] for node in route)
    cursor = start_delay_s
    segments: list[TimedSegment] = []
    for leg_index, (start_node, end_node) in enumerate(zip(route, route[1:])):
        if start_node == end_node:
            continue
        is_cruise = start_node in depot_nodes or end_node in depot_nodes
        mode = "cruise" if is_cruise else "inspection"
        speed = cruise_speed_mps if is_cruise else inspection_speed_mps
        points = data.segment_points(start_node, end_node)
        lengths = [
            float(np.linalg.norm(np.asarray(end) - np.asarray(start)))
            for start, end in zip(points, points[1:])
        ]
        polyline_length = sum(lengths)
        leg_duration = float(data.matrix_m[start_node, end_node]) / speed
        assigned_leg_duration = 0.0
        for segment_index, (start, end, length) in enumerate(
            zip(points, points[1:], lengths)
        ):
            duration = (
                leg_duration * length / polyline_length
                if polyline_length > 1e-12
                else 0.0
            )
            if segment_index == len(lengths) - 1:
                duration = leg_duration - assigned_leg_duration
            segments.append(
                TimedSegment(
                    vehicle_id=vehicle_id,
                    start_s=cursor,
                    end_s=cursor + max(0.0, duration),
                    start=tuple(float(value) for value in start),
                    end=tuple(float(value) for value in end),
                    mode=mode,
                    route_leg_index=leg_index,
                )
            )
            cursor += max(0.0, duration)
            assigned_leg_duration += max(0.0, duration)
        dwell = dwell_seconds.get(end_node, 0.0) if isinstance(dwell_seconds, dict) else dwell_seconds
        if end_node not in depot_nodes and dwell > 0:
            point = tuple(float(value) for value in points[-1])
            segments.append(
                TimedSegment(
                    vehicle_id=vehicle_id,
                    start_s=cursor,
                    end_s=cursor + dwell,
                    start=point,
                    end=point,
                    mode="dwell",
                    route_leg_index=leg_index,
                )
            )
            cursor += dwell
    return TimedTrajectory(
        vehicle_id=vehicle_id,
        route=route,
        route_labels=labels,
        start_delay_s=start_delay_s,
        airborne_duration_s=cursor - start_delay_s,
        segments=tuple(segments),
    )
