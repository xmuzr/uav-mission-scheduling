"""Orchestrates existing modules without original-stage or cached-output dependencies."""
from dataclasses import asdict
from hashlib import sha256
import json
from time import perf_counter
from uav_cooperation.models.request import MissionRequest
from uav_cooperation.environment.voxel_grid import VoxelGrid
from uav_cooperation.environment.collision import CollisionChecker
from uav_cooperation.path_planning.astar.planner import EndpointAStar
from uav_cooperation.path_planning.path_store import PathStore
from uav_cooperation.path_planning.postprocessing.validation import validate_path
from uav_cooperation.task_allocation.nodes import build_nodes
from uav_cooperation.task_allocation.cost_matrix.strategies import euclidean_matrix,candidate_astar_edges
from uav_cooperation.task_allocation.solver import ALLOCATION_METAHEURISTICS,solve_allocation
from uav_cooperation.task_allocation.metrics import route_metrics
from uav_cooperation.constraints.battery import battery_feasible
from uav_cooperation.scheduling.path_data import PathData
from uav_cooperation.scheduling.trajectory import build_timed_trajectory
from uav_cooperation.scheduling.conflicts import detect_conflicts
from uav_cooperation.scheduling.resolution import resolve_with_departure_delays


def input_fingerprint(data):
    return sha256(json.dumps(data,sort_keys=True,separators=(',',':'),allow_nan=False).encode('utf-8')).hexdigest()


def plan_mission(data,progress=None):
    wall_started = perf_counter()
    started = perf_counter()
    request = MissionRequest.from_dict(data)
    from uav_cooperation.realtime_avoidance.api import make_config,load_obstacles
    make_config(request)
    load_obstacles(request.online.get('obstacles',[]))
    nodes = build_nodes(request)
    timings = {'input_model_s':perf_counter()-started}
    started = perf_counter()
    euclidean = euclidean_matrix(nodes.positions)
    matrix = euclidean.copy()
    pre_edges = candidate_astar_edges(euclidean,request.planning['distance_strategy'],request.planning['astar_threshold_m'])
    timings['euclidean_matrix_and_selection_s'] = perf_counter()-started
    started = perf_counter()
    grid = VoxelGrid(request.scene,request.planning['grid_resolution_m'],
                     request.safety['uav_radius_m']+request.safety['obstacle_margin_m'],
                     request.safety['collision_sample_step_m'])
    planner = EndpointAStar(grid,request.planning['max_astar_expansions'],request.planning['heuristic_weight'])
    checker = CollisionChecker(request.scene,request.safety['collision_sample_step_m'])
    timings['grid_and_planner_s'] = perf_counter()-started
    store,details = PathStore(),[]

    def plan_edges(edges,phase):
        for number,(a,b) in enumerate(edges,1):
            if (a,b) in store.paths:
                continue
            edge_started = perf_counter()
            path = planner.plan(nodes.positions[a],nodes.positions[b])
            store.paths[a,b] = path
            details.append({'phase':phase,'start':a,'end':b,'start_label':nodes.labels[a],
                            'end_label':nodes.labels[b],'planning_s':perf_counter()-edge_started,
                            'euclidean_m':float(euclidean[a,b]),'astar_m':path.length_m,
                            'expanded_nodes':path.expanded_nodes,'method':path.method})
            if progress:
                progress(phase,number,len(edges),nodes.labels[a],nodes.labels[b])

    plan_edges(pre_edges,'allocation_matrix')
    for a,b in pre_edges:
        matrix[a,b] = matrix[b,a] = store.paths[a,b].length_m
    timings['allocation_matrix_astar_s'] = sum(d['planning_s'] for d in details)
    started = perf_counter()
    solution = solve_allocation(request,nodes,matrix)
    timings['or_tools_s'] = perf_counter()-started
    selected = sorted({(min(a,b),max(a,b)) for route in solution.routes
                       for a,b in zip(route.route,route.route[1:]) if a != b})
    plan_edges([edge for edge in selected if edge not in store.paths],'selected_edge_completion')
    timings['selected_edge_completion_astar_s'] = sum(d['planning_s'] for d in details if d['phase']=='selected_edge_completion')
    started = perf_counter()
    actual = euclidean.copy()
    for a,b in selected:
        actual[a,b] = actual[b,a] = store.paths[a,b].length_m
    path_data = PathData(nodes.labels,actual,store)
    dwell = {i:task.dwell_seconds for i,task in enumerate(request.tasks,1)}
    routes,trajectories = [],[]
    for drone,estimated in zip(request.drones,solution.routes):
        metrics = route_metrics(estimated.vehicle_id,estimated.route,actual,drone.cruise_speed_mps,
                                drone.inspection_speed_mps,dwell,drone.usable_seconds/60,nodes.depot_nodes)
        points = []
        for a,b in zip(estimated.route,estimated.route[1:]):
            if a == b:
                continue
            leg = store.points(a,b)
            points.extend(leg if not points else leg[1:])
        points = points or [drone.start_m]
        route = {'drone_id':drone.id,**asdict(metrics),
                 'task_ids':[request.tasks[node-1].id for node in estimated.route if node in nodes.task_nodes],
                 'route_labels':[nodes.labels[node] for node in estimated.route],
                 'start_m':drone.start_m,'end_m':drone.end_m,'points_m':points,
                 'usable_flight_seconds':drone.usable_seconds,
                 'departure_delay_s':0.0,'completion_time_s':metrics.task_time_s}
        route['battery_feasible'] = battery_feasible(drone,metrics.task_time_s)
        routes.append(route)
        trajectory = build_timed_trajectory(path_data,asdict(metrics),nodes.depot_nodes,
                                            drone.cruise_speed_mps,drone.inspection_speed_mps,dwell)
        if trajectory.airborne_duration_s > 1e-9:
            trajectories.append(trajectory)
    geometric_ok = all(validate_path(store.paths[a,b],nodes.positions[a],nodes.positions[b],checker) for a,b in selected)
    visits = [node for route in solution.routes for node in route.route if node in nodes.task_nodes]
    validation = {'all_tasks_once':sorted(visits)==sorted(nodes.task_nodes),
                  'all_paths_endpoints_lengths_and_sampled_collisions_pass':geometric_ok,
                  'all_routes_within_battery':all(route['battery_feasible'] for route in routes)}
    timings['assembly_and_validation_s'] = perf_counter()-started
    offline_makespan = max(route['task_time_s'] for route in routes)
    started = perf_counter()
    before = detect_conflicts(tuple(trajectories),request.safety['minimum_separation_m'])
    schedule = resolve_with_departure_delays(tuple(trajectories),request.safety['minimum_separation_m'],
                                             request.safety['delay_step_seconds'],request.safety['maximum_departure_delay_seconds'])
    after = detect_conflicts(schedule.trajectories,request.safety['minimum_separation_m'])
    validation['zero_inter_uav_conflicts_after_scheduling'] = not after
    for trajectory in schedule.trajectories:
        route = routes[trajectory.vehicle_id-1]
        route['departure_delay_s'] = trajectory.start_s
        route['completion_time_s'] = trajectory.end_s
    timings['conflict_detection_and_scheduling_s'] = perf_counter()-started
    pre_schedule_keys = ('input_model_s','euclidean_matrix_and_selection_s','grid_and_planner_s',
                         'allocation_matrix_astar_s','or_tools_s','selected_edge_completion_astar_s','assembly_and_validation_s')
    timings['planning_compute_s_before_scheduling'] = sum(timings[key] for key in pre_schedule_keys)
    timings['wall_s_including_scheduling_and_progress'] = perf_counter()-wall_started
    feasible = all(validation.values())
    return {'schema_version':'1.0','name':request.name,'coordinate_frame':'local_xyz_m','input_fingerprint':input_fingerprint(data),
            'distance_strategy':request.planning['distance_strategy'],'astar_threshold_m':request.planning['astar_threshold_m'],
            'status':'feasible' if feasible else 'infeasible_actual_routes','feasible':feasible,
            'reassignment_required':not validation['all_routes_within_battery'],'validation':validation,
            'drone_count':len(request.drones),'task_count':len(request.tasks),'timing_s':timings,
            'allocation':{'strategy':request.planning['allocation_strategy'],'solver':solution.solver,
                          'first_solution_strategy':'PARALLEL_CHEAPEST_INSERTION',
                          'local_search_metaheuristic':ALLOCATION_METAHEURISTICS[request.planning['allocation_strategy']],
                          'status':solution.status,'objective_value':solution.objective_value,
                          'search_time_s':solution.runtime_s,'optimality_proven':False},
            'preallocation_astar_edge_count':len(pre_edges),
            'postallocation_new_astar_edge_count':len(store.paths)-len(pre_edges),
            'total_astar_calls':len(store.paths),'selected_unique_edge_count':len(selected),
            'estimated_makespan_s':solution.makespan_s,'actual_makespan_before_scheduling_s':offline_makespan,
            'makespan_after_scheduling_s':schedule.makespan_s,'sum_actual_distance_m':sum(route['distance_m'] for route in routes),
            'routes':routes,'priority_order':schedule.priority_order,
            'conflicts_before':[c.to_dict() for c in before],'conflicts_after':[c.to_dict() for c in after],
            'trajectories':[asdict(t) for t in schedule.trajectories],
            'node_labels':nodes.labels,'node_positions_m':nodes.positions,
            'allocation_matrix_m':matrix.tolist(),'edge_details':details,
            'selected_paths':[{'start':a,'end':b,'points_m':store.paths[a,b].points,'length_m':store.paths[a,b].length_m} for a,b in selected],
            'notes':['Only Weighted A* is used for final geometric paths.',
                     'Primitive obstacles use sampled segment checks; closed triangle meshes also use triangle intersection and swept-sphere distance checks. These numerical checks are not a real-flight safety proof.',
                     'Available flight time minus reserve is the battery model; no percentage conversion.',
                     'Actual over-budget routes are rejected; automatic task reassignment is not implemented.',
                     'Schedules enumerate priorities up to six active drones; larger fleets use four heuristic orders.']}
