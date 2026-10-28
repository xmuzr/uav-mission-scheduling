"""Endpoint adapter around the migrated Weighted A* implementation."""
from itertools import product
import numpy as np
from .search import AStarPlanner,PathPlan


class EndpointAStar(AStarPlanner):
    def connector(self,point):
        center = np.rint((np.asarray(point)-self.grid.map_min)/self.grid.resolution_m).astype(int)
        for radius in range(4):
            candidates = []
            for delta in product(range(-radius,radius+1),repeat=3):
                index = tuple(int(center[i]+delta[i]) for i in range(3))
                if self.grid.is_free_index(index):
                    value = self.grid.index_to_point(index)
                    candidates.append((float(np.linalg.norm(np.asarray(value)-point)),index,value))
            for _,index,value in sorted(candidates):
                if self.grid.line_is_free(point,value):
                    return index
        raise RuntimeError(f'No free endpoint connector near {point}; use a finer grid')

    def plan(self,start_point,goal_point):
        start_point,goal_point = tuple(start_point),tuple(goal_point)
        # Grid-aligned input follows the original algorithm unchanged.
        aligned = all(np.allclose((np.asarray(p)-self.grid.map_min)/self.grid.resolution_m,
                                  np.rint((np.asarray(p)-self.grid.map_min)/self.grid.resolution_m),atol=1e-8,rtol=0)
                      for p in (start_point,goal_point))
        if aligned:
            return super().plan(start_point,goal_point)
        if not self.grid.point_is_free(start_point) or not self.grid.point_is_free(goal_point):
            raise ValueError('Endpoint is occupied or outside the map')
        if self.grid.line_is_free(start_point,goal_point):
            return PathPlan((start_point,goal_point),self._length([start_point,goal_point]),2,2,0,'direct_line_of_sight')
        start,goal = self.connector(start_point),self.connector(goal_point)
        raw,expanded = self._search(start,goal)
        simplified = self._simplify(raw)
        points = [start_point,*(self.grid.index_to_point(i) for i in simplified),goal_point]
        points = [p for i,p in enumerate(points) if i == 0 or p != points[i-1]]
        return PathPlan(tuple(points),self._length(points),len(raw),len(points),expanded,'astar_with_endpoint_connectors')
