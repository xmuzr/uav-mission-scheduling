"""OR-Tools min-max allocation: parallel cheapest insertion plus greedy descent by default."""
from math import ceil,floor
from time import perf_counter
import numpy as np
from ortools.constraint_solver import pywrapcp,routing_enums_pb2
from .metrics import route_metrics,summarize_routes


ALLOCATION_METAHEURISTICS = {'pci_greedy':'GREEDY_DESCENT','pci_gls':'GUIDED_LOCAL_SEARCH'}


def solve_allocation(request,nodes,matrix):
    if matrix.shape != (len(nodes.positions),len(nodes.positions)) or not np.all(np.isfinite(matrix)):
        raise ValueError('Allocation matrix shape or values are invalid')
    manager = pywrapcp.RoutingIndexManager(len(nodes.positions),len(request.drones),list(nodes.starts),list(nodes.ends))
    routing = pywrapcp.RoutingModel(manager)
    dwell = {i:task.dwell_seconds for i,task in enumerate(request.tasks,1)}
    scale = 1000
    callbacks = []
    transit_indices = []
    registered = {}
    depot_nodes = nodes.depot_nodes
    for vehicle,drone in enumerate(request.drones):
        key = drone.cruise_speed_mps,drone.inspection_speed_mps
        if key not in registered:
            def callback(a,b,cruise=drone.cruise_speed_mps,inspection=drone.inspection_speed_mps):
                start,end = manager.IndexToNode(a),manager.IndexToNode(b)
                speed = cruise if start in depot_nodes or end in depot_nodes else inspection
                return ceil((float(matrix[start,end])/speed+dwell.get(end,0))*scale-1e-9)
            callbacks.append(callback)
            registered[key] = routing.RegisterTransitCallback(callback)
        index = registered[key]
        transit_indices.append(index)
        routing.SetArcCostEvaluatorOfVehicle(index,vehicle)
    budgets = [floor(drone.usable_seconds*scale) for drone in request.drones]
    routing.AddDimensionWithVehicleTransitAndCapacity(transit_indices,0,budgets,True,'TaskTime')
    routing.GetDimensionOrDie('TaskTime').SetGlobalSpanCostCoefficient(10000)
    search = pywrapcp.DefaultRoutingSearchParameters()
    search.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PARALLEL_CHEAPEST_INSERTION
    strategy = request.planning['allocation_strategy']
    search.local_search_metaheuristic = getattr(routing_enums_pb2.LocalSearchMetaheuristic,
                                               ALLOCATION_METAHEURISTICS[strategy])
    search.time_limit.FromMilliseconds(max(1,round(request.planning['solver_time_limit_seconds']*1000)))
    search.solution_limit = request.planning['solution_limit']
    search.sat_parameters.random_seed = request.planning['seed']
    search.sat_parameters.num_search_workers = 1
    started = perf_counter()
    solution = routing.SolveWithParameters(search)
    elapsed = perf_counter()-started
    if solution is None:
        raise RuntimeError('No feasible allocation found within the solver limit; check tasks and flight budgets')
    routes = []
    for vehicle,drone in enumerate(request.drones):
        index = routing.Start(vehicle)
        route = [manager.IndexToNode(index)]
        while not routing.IsEnd(index):
            index = solution.Value(routing.NextVar(index))
            route.append(manager.IndexToNode(index))
        routes.append(route_metrics(vehicle+1,tuple(route),matrix,drone.cruise_speed_mps,
                                    drone.inspection_speed_mps,dwell,drone.usable_seconds/60,nodes.depot_nodes))
    if request.planning['require_all_drones_used'] and any(route.viewpoint_count == 0 for route in routes):
        raise RuntimeError('Allocation leaves a drone unused although require_all_drones_used is enabled')
    return summarize_routes(tuple(routes),f'google_or_tools_routing_minmax_mtsp_{strategy}',
                            f'ROUTING_STATUS_{int(routing.status())}',elapsed,int(solution.ObjectiveValue()))
