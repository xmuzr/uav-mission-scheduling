"""Single-cycle avoidance adapter. This is a planning command, not flight control."""
from dataclasses import asdict
import numpy as np
from uav_cooperation.models.request import MissionRequest,check_keys,number,point,identifier
from uav_cooperation.environment.collision import CollisionChecker
from .config import OnlineAvoidanceConfig
from .obstacles import SphericalObstacle
from .local_avoidance import NeighborState,choose_safe_velocity


def make_config(request):
    values = {key:value for key,value in request.online.items() if key != 'obstacles'}
    return OnlineAvoidanceConfig(**values,uav_collision_radius_m=request.safety['uav_radius_m'],
                                 static_safety_margin_m=request.safety['obstacle_margin_m'],
                                 minimum_inter_uav_separation_m=request.safety['minimum_separation_m'])


def load_obstacles(raw_obstacles):
    if not isinstance(raw_obstacles,list):
        raise ValueError('online.obstacles must be a list')
    obstacles = []
    for raw in raw_obstacles:
        check_keys(raw,('id','kind','radius_m','center_m','velocity_mps','active_start_seconds','active_end_seconds'),'online obstacle')
        if raw.get('kind') not in ('static','moving'):
            raise ValueError('Online obstacle kind must be static or moving')
        velocity = point(raw.get('velocity_mps',[0,0,0]),'obstacle velocity')
        if raw['kind'] == 'static' and any(velocity):
            raise ValueError('Static obstacles cannot have velocity')
        end = raw.get('active_end_seconds')
        obstacles.append(SphericalObstacle(identifier(raw['id'],'obstacle.id'),raw['kind'],
                                           number(raw['radius_m'],'radius_m',positive=True),point(raw['center_m'],'center_m'),
                                           velocity,number(raw.get('active_start_seconds',0),'active_start_seconds'),
                                           float('inf') if end is None else number(end,'active_end_seconds')))
    if len({o.obstacle_id for o in obstacles}) != len(obstacles):
        raise ValueError('Online obstacle IDs must be unique')
    return tuple(obstacles)


def safe_velocity(data,*,drone_id,time_seconds,position_m,target_m,preferred_velocity_mps,
                  previous_velocity_mps=(0,0,0),neighbors=()):
    request = MissionRequest.from_dict(data)
    ids = [drone.id for drone in request.drones]
    if drone_id not in ids:
        raise ValueError('Unknown drone_id')
    vehicle = request.drones[ids.index(drone_id)]
    time_seconds = number(time_seconds,'time_seconds')
    position_m,target_m = point(position_m,'position_m'),point(target_m,'target_m')
    preferred_velocity_mps = point(preferred_velocity_mps,'preferred_velocity_mps')
    previous_velocity_mps = point(previous_velocity_mps,'previous_velocity_mps')
    if np.linalg.norm(preferred_velocity_mps) > max(vehicle.cruise_speed_mps,vehicle.inspection_speed_mps)+1e-8:
        raise ValueError('Preferred velocity exceeds configured speed')
    states = []
    for raw in neighbors:
        if raw['drone_id'] not in ids:
            raise ValueError('Unknown neighbor drone_id')
        states.append(NeighborState(ids.index(raw['drone_id'])+1,point(raw['position_m'],'neighbor position'),
                                    point(raw['velocity_mps'],'neighbor velocity'),raw.get('active',True)))
    checker = CollisionChecker(request.scene,request.safety['collision_sample_step_m'])
    return asdict(choose_safe_velocity(vehicle_id=ids.index(drone_id)+1,time_s=time_seconds,
                                      position_m=position_m,target_m=target_m,preferred_velocity_mps=preferred_velocity_mps,
                                      previous_velocity_mps=previous_velocity_mps,
                                      obstacles=load_obstacles(request.online.get('obstacles',[])),neighbors=tuple(states),
                                      base_point_is_free=checker.point_is_free,base_segment_is_free=checker.line_is_free,
                                      config=make_config(request)))
