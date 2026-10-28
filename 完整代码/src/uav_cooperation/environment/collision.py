"""Mesh swept-sphere checks and original sampled primitive checks, without a grid."""
from math import ceil
import numpy as np
from .mesh import TriangleMesh


def segment_is_free(scene,start,end,clearance_m,sample_step_m):
    start,end=np.asarray(start,dtype=float),np.asarray(end,dtype=float)
    if np.any(np.minimum(start,end)<scene.config.map_min) or np.any(np.maximum(start,end)>scene.config.map_max):
        return False
    for obstacle in scene.obstacles:
        if isinstance(obstacle,TriangleMesh) and not obstacle.segment_is_free(start,end,clearance_m):
            return False
    primitives=[o for o in scene.obstacles if not isinstance(o,TriangleMesh)]
    if not primitives:return True
    count=max(1,ceil(float(np.linalg.norm(end-start))/sample_step_m))
    return all(not any(o.contains(start+ratio*(end-start),margin=clearance_m) for o in primitives)
               for ratio in np.linspace(0,1,count+1))


class CollisionChecker:
    def __init__(self,scene,sample_step_m):
        self.scene = scene
        self.sample_step_m = sample_step_m
        self.clearance_m = scene.config.drone_radius_m+scene.config.safety_margin_m

    def point_is_free(self,point):
        if any(x<a or x>b for x,a,b in zip(point,self.scene.config.map_min,self.scene.config.map_max)):
            return False
        return not any(o.contains(tuple(point),margin=self.clearance_m) for o in self.scene.obstacles)

    def line_is_free(self,start,end):
        return segment_is_free(self.scene,start,end,self.clearance_m,self.sample_step_m)
