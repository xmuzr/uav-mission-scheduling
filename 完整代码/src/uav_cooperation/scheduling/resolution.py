from __future__ import annotations

from dataclasses import dataclass
from itertools import permutations
from math import floor

from .conflicts import detect_pair_conflicts
from .trajectory import TimedTrajectory


@dataclass(frozen=True)
class ResolutionResult:
    trajectories: tuple[TimedTrajectory, ...]
    priority_order: tuple[int, ...]
    departure_delays_s: tuple[float, ...]
    makespan_s: float


def resolve_with_departure_delays(
    trajectories: tuple[TimedTrajectory, ...],
    minimum_separation_m: float,
    delay_step_s: float,
    maximum_delay_s: float,
) -> ResolutionResult:
    by_vehicle = {trajectory.vehicle_id: trajectory.shifted(0.0) for trajectory in trajectories}
    vehicle_ids = tuple(sorted(by_vehicle))
    best: tuple[tuple[float, float, tuple[float, ...], tuple[int, ...]], ResolutionResult] | None = None
    if not vehicle_ids:
        return ResolutionResult((), (), (), 0.0)
    steps = floor(maximum_delay_s / delay_step_s)
    if len(vehicle_ids) <= 6:
        priority_orders = permutations(vehicle_ids)
    else:
        longest_first = tuple(sorted(vehicle_ids, key=lambda v: -by_vehicle[v].airborne_duration_s))
        priority_orders = dict.fromkeys((vehicle_ids, tuple(reversed(vehicle_ids)),
                                        longest_first, tuple(reversed(longest_first))))
    for priority_order in priority_orders:
        scheduled: dict[int, TimedTrajectory] = {}
        feasible = True
        for vehicle_id in priority_order:
            selected = None
            for step in range(steps + 1):
                delay = step * delay_step_s
                candidate = by_vehicle[vehicle_id].shifted(delay)
                if all(
                    not detect_pair_conflicts(
                        candidate,
                        other,
                        minimum_separation_m,
                        stop_after_first=True,
                    )
                    for other in scheduled.values()
                ):
                    selected = candidate
                    break
            if selected is None:
                feasible = False
                break
            scheduled[vehicle_id] = selected
        if not feasible:
            continue
        ordered = tuple(scheduled[vehicle_id] for vehicle_id in vehicle_ids)
        delays = tuple(trajectory.start_delay_s for trajectory in ordered)
        result = ResolutionResult(
            trajectories=ordered,
            priority_order=priority_order,
            departure_delays_s=delays,
            makespan_s=max(trajectory.end_s for trajectory in ordered),
        )
        key = (result.makespan_s, sum(delays), delays, priority_order)
        if best is None or key < best[0]:
            best = (key, result)
    if best is None:
        raise RuntimeError("No conflict-free departure-delay schedule was found")
    return best[1]

