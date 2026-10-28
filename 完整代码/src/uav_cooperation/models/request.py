"""Public input contract: local metric coordinates and time-based batteries."""
from dataclasses import dataclass
from math import isfinite
from .geometry import BridgeScenario,Cuboid,CylinderZ,PlanningConfig,Viewpoint
from uav_cooperation.environment.mesh import TriangleMesh


def check_keys(value,allowed,name):
    if not isinstance(value,dict):
        raise ValueError(f'{name} must be an object')
    unknown = set(value)-set(allowed)
    if unknown:
        raise ValueError(f'Unknown {name} fields: {sorted(unknown)}')


def number(value,name,minimum=0,positive=False):
    if isinstance(value,bool) or not isinstance(value,(int,float)) or not isfinite(value):
        raise ValueError(f'{name} must be a finite number')
    if value < minimum or (positive and value <= minimum):
        raise ValueError(f'Invalid {name}: {value}')
    return float(value)


def integer(value,name,minimum=1):
    if isinstance(value,bool) or not isinstance(value,int) or value < minimum:
        raise ValueError(f'{name} must be an integer >= {minimum}')
    return value


def point(value,name):
    if not isinstance(value,(list,tuple)) or len(value) != 3:
        raise ValueError(f'{name} must contain XYZ coordinates')
    return tuple(number(x,name,minimum=-float('inf')) for x in value)


def identifier(value,name):
    if not isinstance(value,str) or not value.strip():
        raise ValueError(f'{name} must be a nonempty string')
    return value


@dataclass(frozen=True)
class Drone:
    id: str
    start_m: tuple
    end_m: tuple
    available_flight_seconds: float
    reserve_seconds: float
    cruise_speed_mps: float
    inspection_speed_mps: float

    @property
    def usable_seconds(self):
        return self.available_flight_seconds-self.reserve_seconds


@dataclass(frozen=True)
class Task:
    id: str
    position_m: tuple
    dwell_seconds: float


@dataclass(frozen=True)
class MissionRequest:
    name: str
    drones: tuple
    tasks: tuple
    scene: BridgeScenario
    planning: dict
    safety: dict
    online: dict

    @classmethod
    def from_dict(cls,data):
        check_keys(data,('schema_version','name','coordinate_frame','drone_count','drones','tasks',
                         'environment','planning','safety','online'),'mission')
        if data.get('schema_version','1.0') != '1.0':
            raise ValueError('Only schema_version 1.0 is supported')
        if data.get('coordinate_frame','local_xyz_m') != 'local_xyz_m':
            raise ValueError('Convert geographic coordinates to local_xyz_m first')
        if not isinstance(data.get('drones'),list) or not data['drones']:
            raise ValueError('drones must be a nonempty list')
        if not isinstance(data.get('tasks'),list) or not data['tasks']:
            raise ValueError('tasks must be a nonempty list')
        count = integer(data.get('drone_count',len(data['drones'])),'drone_count')
        if count != len(data['drones']):
            raise ValueError('drone_count differs from the drone list length')
        drones = []
        for raw in data['drones']:
            check_keys(raw,('id','start_m','end_m','available_flight_seconds','reserve_seconds',
                            'cruise_speed_mps','inspection_speed_mps'),'drone')
            start = point(raw['start_m'],'start_m')
            speed = number(raw['cruise_speed_mps'],'cruise_speed_mps',positive=True)
            drone = Drone(identifier(raw['id'],'drone.id'),start,point(raw.get('end_m',start),'end_m'),
                          number(raw['available_flight_seconds'],'available_flight_seconds',positive=True),
                          number(raw.get('reserve_seconds',0),'reserve_seconds'),speed,
                          number(raw.get('inspection_speed_mps',speed),'inspection_speed_mps',positive=True))
            if drone.usable_seconds <= 0:
                raise ValueError('Battery reserve consumes all available flight time')
            drones.append(drone)
        tasks = []
        for raw in data['tasks']:
            check_keys(raw,('id','position_m','dwell_seconds'),'task')
            tasks.append(Task(identifier(raw['id'],'task.id'),point(raw['position_m'],'position_m'),
                              number(raw.get('dwell_seconds',2),'dwell_seconds')))
        if len({d.id for d in drones}) != len(drones) or len({t.id for t in tasks}) != len(tasks):
            raise ValueError('Drone IDs and task IDs must each be unique')
        if len({t.position_m for t in tasks}) != len(tasks):
            raise ValueError('Task coordinates must be unique; merge tasks at the same position')
        planning = dict(distance_strategy='hybrid',astar_threshold_m=600.0,grid_resolution_m=10.0,
                        heuristic_weight=1.2,max_astar_expansions=2_000_000,
                        allocation_strategy='pci_greedy',solver_time_limit_seconds=30.0,solution_limit=10000,seed=20260910,
                        require_all_drones_used=False)
        check_keys(data.get('planning',{}),planning,'planning')
        planning.update(data.get('planning',{}))
        if planning['distance_strategy'] not in ('euclidean','hybrid','astar'):
            raise ValueError('distance_strategy must be euclidean, hybrid or astar')
        if planning['allocation_strategy'] not in ('pci_greedy','pci_gls'):
            raise ValueError('allocation_strategy must be pci_greedy or pci_gls')
        for key in ('astar_threshold_m','grid_resolution_m','solver_time_limit_seconds'):
            planning[key] = number(planning[key],key,positive=True)
        planning['heuristic_weight'] = number(planning['heuristic_weight'],'heuristic_weight',minimum=1)
        for key in ('max_astar_expansions','solution_limit','seed'):
            planning[key] = integer(planning[key],key,0 if key == 'seed' else 1)
        if not isinstance(planning['require_all_drones_used'],bool):
            raise ValueError('require_all_drones_used must be boolean')
        if planning['require_all_drones_used'] and len(tasks) < count:
            raise ValueError('Not enough tasks to use every drone')
        safety = dict(uav_radius_m=0.5,obstacle_margin_m=0.0,minimum_separation_m=20.0,
                      collision_sample_step_m=5.0,delay_step_seconds=1.0,
                      maximum_departure_delay_seconds=600.0)
        check_keys(data.get('safety',{}),safety,'safety')
        safety.update(data.get('safety',{}))
        for key in safety:
            safety[key] = number(safety[key],key,positive=key in ('uav_radius_m','collision_sample_step_m','delay_step_seconds'))
        if safety['minimum_separation_m'] < 2*safety['uav_radius_m']:
            raise ValueError('Inter-UAV separation cannot be smaller than the physical diameter')
        if safety['maximum_departure_delay_seconds']/safety['delay_step_seconds'] > 10000:
            raise ValueError('Departure search exceeds 10000 steps; increase delay_step_seconds')
        environment = data.get('environment')
        check_keys(environment,('map_min_m','map_max_m','obstacles'),'environment')
        low,high = point(environment['map_min_m'],'map_min_m'),point(environment['map_max_m'],'map_max_m')
        if any(b <= a for a,b in zip(low,high)):
            raise ValueError('Every map_max_m coordinate must exceed map_min_m')
        resolution = planning['grid_resolution_m']
        if any(abs((b-a)/resolution-round((b-a)/resolution))>1e-8 for a,b in zip(low,high)):
            raise ValueError('Map extents must be divisible by grid_resolution_m')
        size = 1
        for a,b in zip(low,high):
            size *= round((b-a)/resolution)+1
        if size > 20_000_000:
            raise ValueError('Grid exceeds 20 million nodes; use a coarser resolution')
        obstacles = []
        if not isinstance(environment.get('obstacles',[]),list):
            raise ValueError('environment.obstacles must be a list')
        for raw in environment.get('obstacles',[]):
            kind = raw.get('type')
            if kind == 'triangle_mesh':
                check_keys(raw,('type','vertices_m','faces','label'),'mesh obstacle')
                obstacles.append(TriangleMesh(raw['vertices_m'],raw['faces'],raw.get('label','triangle_mesh')))
                continue
            check_keys(raw,('type','center_m','size_m','label') if kind == 'cuboid' else
                       ('type','center_m','radius_m','height_m','label'),'obstacle')
            center = point(raw['center_m'],'obstacle.center_m')
            if kind == 'cuboid':
                dimensions = point(raw['size_m'],'size_m')
                if min(dimensions) <= 0:
                    raise ValueError('Cuboid dimensions must be positive')
                obstacles.append(Cuboid(center,dimensions,raw.get('label','obstacle')))
            elif kind == 'cylinder_z':
                obstacles.append(CylinderZ(center,number(raw['radius_m'],'radius_m',positive=True),
                                           number(raw['height_m'],'height_m',positive=True),raw.get('label','obstacle')))
            else:
                raise ValueError('Supported obstacles: cuboid, cylinder_z, triangle_mesh')
        clearance = safety['uav_radius_m']+safety['obstacle_margin_m']
        for coordinate in [*(t.position_m for t in tasks),*(d.start_m for d in drones),*(d.end_m for d in drones)]:
            if any(x<a or x>b for x,a,b in zip(coordinate,low,high)):
                raise ValueError(f'Point outside planning bounds: {coordinate}')
            if any(o.contains(coordinate,margin=clearance) for o in obstacles):
                raise ValueError(f'Point lies inside an inflated obstacle: {coordinate}')
        online = dict(data.get('online',{}))
        check_keys(online,('planning_update_hz','prediction_horizon_s','sensor_range_m',
                           'maximum_simulation_overtime_s','obstacles'),'online')
        for key in ('planning_update_hz','prediction_horizon_s','sensor_range_m','maximum_simulation_overtime_s'):
            if key in online:
                online[key] = number(online[key],key,positive=True)
        scene = BridgeScenario(data.get('name','custom_mission'),tuple(obstacles),
                               tuple(Viewpoint(i,t.position_m,'custom') for i,t in enumerate(tasks,1)),
                               drones[0].start_m,count,
                               PlanningConfig(map_min=low,map_max=high,grid_resolution_m=resolution,
                                              drone_radius_m=safety['uav_radius_m'],safety_margin_m=safety['obstacle_margin_m']))
        return cls(scene.name,tuple(drones),tuple(tasks),scene,planning,safety,online)
