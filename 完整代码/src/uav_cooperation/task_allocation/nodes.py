from dataclasses import dataclass


@dataclass(frozen=True)
class PlanningNodes:
    positions: tuple
    labels: tuple
    starts: tuple
    ends: tuple
    task_nodes: frozenset

    @property
    def depot_nodes(self):
        return set(range(len(self.positions)))-self.task_nodes


def build_nodes(request):
    # Preserve original S1, tasks, additional depots ordering for the bridge.
    positions = [request.drones[0].start_m,*(task.position_m for task in request.tasks)]
    labels = ['S1',*(task.id for task in request.tasks)]
    depot_lookup = {request.drones[0].start_m:0}
    starts,ends = [],[]
    for drone in request.drones:
        for coordinate,destination in ((drone.start_m,starts),(drone.end_m,ends)):
            if coordinate not in depot_lookup:
                depot_lookup[coordinate] = len(positions)
                positions.append(coordinate)
                labels.append(f'S{len(depot_lookup)}')
            destination.append(depot_lookup[coordinate])
    return PlanningNodes(tuple(positions),tuple(labels),tuple(starts),tuple(ends),frozenset(range(1,len(request.tasks)+1)))
