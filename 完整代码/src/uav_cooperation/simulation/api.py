from dataclasses import asdict
from uav_cooperation.models.request import MissionRequest
from uav_cooperation.application.pipeline import input_fingerprint
from uav_cooperation.environment.collision import CollisionChecker
from uav_cooperation.scheduling.trajectory import TimedSegment,TimedTrajectory
from uav_cooperation.realtime_avoidance.api import make_config,load_obstacles
from .online import OnlineScenario,simulate_online_scenario


def simulate_mission(data,plan):
    request = MissionRequest.from_dict(data)
    if not plan.get('feasible'):
        raise ValueError('Simulation requires a feasible offline plan')
    if plan.get('input_fingerprint') != input_fingerprint(data):
        raise ValueError('Plan does not match inputs; use its exported mission.json or replan')
    trajectories = tuple(TimedTrajectory(vehicle_id=raw['vehicle_id'],route=tuple(raw['route']),route_labels=tuple(raw['route_labels']),
                                         start_delay_s=raw['start_delay_s'],airborne_duration_s=raw['airborne_duration_s'],
                                         segments=tuple(TimedSegment(**s) for s in raw['segments'])) for raw in plan['trajectories'])
    instantaneous = {task for route in plan['routes'] if route['task_time_s']<=1e-9 for task in route['task_ids']}
    if not trajectories:
        return {'status':'feasible','feasible':True,'coverage_count':len(instantaneous),
                'completion_time_s':0.0,'collision_count':0,'inter_uav_conflict_count':0,
                'battery_feasible_by_drone':{drone.id:True for drone in request.drones},'trajectory_rows':[],'planning_rows':[]}
    scenario = OnlineScenario(request.name,'User-specified online obstacles',load_obstacles(request.online.get('obstacles',[])))
    checker = CollisionChecker(request.scene,request.safety['collision_sample_step_m'])
    result = simulate_online_scenario(scenario,trajectories,checker,make_config(request),tuple(plan['priority_order']),set(range(1,len(request.tasks)+1)))
    payload = asdict(result)
    payload['scenario']['obstacles'] = [o.to_dict() for o in scenario.obstacles]
    payload['coverage_count'] += len(instantaneous)
    budget_checks = {drone.id:result.airborne_durations_s.get(index,0.0) <= drone.usable_seconds+1e-8
                     for index,drone in enumerate(request.drones,1)}
    payload['battery_feasible_by_drone'] = budget_checks
    payload['feasible'] = result.completed and payload['coverage_count']==len(request.tasks) and not result.collision_count and not result.inter_uav_conflict_count and all(budget_checks.values())
    payload['status'] = 'feasible' if payload['feasible'] else 'infeasible_online_execution'
    return payload
