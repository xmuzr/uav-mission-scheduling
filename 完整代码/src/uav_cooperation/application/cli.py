import argparse
import json
from pathlib import Path
import sys
from .pipeline import plan_mission
from uav_cooperation.models.request import MissionRequest
from uav_cooperation.io.exports import export_plan,write_json


def main(argv=None):
    parser = argparse.ArgumentParser(description='Multi-UAV task allocation, A* planning and online avoidance')
    parser.add_argument('command',choices=('validate','plan','simulate','view'))
    parser.add_argument('mission',type=Path)
    parser.add_argument('--output',type=Path,default=Path('outputs/latest'))
    parser.add_argument('--plan',type=Path,help='Exported plan for simulate/view')
    parser.add_argument('--strategy',choices=('euclidean','hybrid','astar'))
    parser.add_argument('--threshold',type=float)
    parser.add_argument('--solver-seconds',type=float)
    parser.add_argument('--allocation-strategy',choices=('pci_greedy','pci_gls'),
                        help='Parallel cheapest insertion plus greedy descent (default) or GLS')
    parser.add_argument('--snapshot',type=Path)
    args = parser.parse_args(argv)
    try:
        data = json.loads(args.mission.read_text(encoding='utf-8'))
        options = data.setdefault('planning',{})
        for key,value in (('distance_strategy',args.strategy),('astar_threshold_m',args.threshold),
                          ('solver_time_limit_seconds',args.solver_seconds),('allocation_strategy',args.allocation_strategy)):
            if value is not None:
                options[key] = value
        if args.command == 'validate':
            request = MissionRequest.from_dict(data)
            from uav_cooperation.realtime_avoidance.api import make_config,load_obstacles
            make_config(request)
            load_obstacles(request.online.get('obstacles',[]))
            print(json.dumps({'valid':True,'drones':len(request.drones),'tasks':len(request.tasks),'obstacles':len(request.scene.obstacles)}))
            return 0
        if args.plan:
            plan = json.loads(args.plan.read_text(encoding='utf-8'))
            if args.command == 'plan':
                raise ValueError('--plan is only for simulate/view')
        else:
            def progress(phase,number,total,start,end):
                if number%10 == 0 or number==total:
                    print(f'{phase}: {number}/{total}; {start}->{end}',flush=True)
            plan = plan_mission(data,progress=progress)
            export_plan(args.output,data,plan)
        if args.command == 'simulate':
            from uav_cooperation.simulation.api import simulate_mission
            result = simulate_mission(data,plan)
            write_json(args.output/'online_simulation.json',result)
            print(json.dumps({key:result[key] for key in ('status','coverage_count','completion_time_s','collision_count','inter_uav_conflict_count','battery_feasible_by_drone')}))
            return 0 if result['feasible'] else 2
        if args.command == 'view' or args.snapshot:
            from uav_cooperation.visualization.viewer import show_plan
            if args.snapshot:
                args.snapshot.parent.mkdir(parents=True,exist_ok=True)
            show_plan(data,plan,args.snapshot)
        print(json.dumps({key:plan[key] for key in ('status','drone_count','task_count','allocation','preallocation_astar_edge_count',
                         'postallocation_new_astar_edge_count','total_astar_calls','actual_makespan_before_scheduling_s',
                         'makespan_after_scheduling_s','timing_s') if key in plan}))
        return 0 if plan['feasible'] else 2
    except (ValueError,RuntimeError,KeyError,TypeError,OSError) as error:
        write_json(args.output/'error.json',{'status':'rejected','error':str(error)})
        print(f'Error: {error}',file=sys.stderr)
        return 2
