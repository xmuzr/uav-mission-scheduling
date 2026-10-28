"""Check the release inputs and run the actual planner/simulator on a small scene."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import platform
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / '完整代码'
sys.path.insert(0, str(SOURCE / 'src'))


def require(condition, message):
    if not condition:
        raise ValueError(message)


def load(path):
    return json.loads(path.read_text(encoding='utf-8'))


def check_scene(name, folder=None):
    from uav_cooperation.application.pipeline import input_fingerprint
    from uav_cooperation.models.request import MissionRequest
    from uav_cooperation.realtime_avoidance.api import make_config, load_obstacles

    original = load(ROOT / '数据集' / name / 'mission.json')
    request = MissionRequest.from_dict(original)
    make_config(request)
    load_obstacles(request.online.get('obstacles', []))
    folder = folder or ROOT / '数据集' / '运行结果' / name
    mission = load(folder / 'mission.json')
    plan = load(folder / 'plan.json')
    simulation = load(folder / 'online_simulation.json')
    require(mission == original, f'{name}: preset input differs from the scenario')
    require(plan['input_fingerprint'] == input_fingerprint(mission), f'{name}: stale plan')
    require(plan['feasible'] and all(plan['validation'].values()), f'{name}: rejected plan')
    actual_tasks = [task for route in plan['routes'] for task in route['task_ids']]
    require(Counter(actual_tasks) == Counter(task['id'] for task in mission['tasks']),
            f'{name}: tasks must be assigned exactly once')
    require(simulation['feasible'] and simulation['completed'], f'{name}: simulation incomplete')
    require(simulation['coverage_count'] == len(mission['tasks']), f'{name}: incomplete visits')
    require(simulation['collision_count'] == simulation['inter_uav_conflict_count'] == 0,
            f'{name}: simulation collision or inter-UAV conflict')
    require(all(simulation['battery_feasible_by_drone'].values()), f'{name}: battery infeasible')
    return {
        'scene': name, 'drones': len(mission['drones']), 'tasks': len(mission['tasks']),
        'input_fingerprint_matches': True, 'all_tasks_once': True,
        'distance_m': plan['sum_actual_distance_m'],
        'scheduled_completion_s': plan['makespan_after_scheduling_s'],
        'simulation_completion_s': simulation['completion_time_s'],
        'coverage_count': simulation['coverage_count'],
        'collision_count': simulation['collision_count'],
        'inter_uav_conflict_count': simulation['inter_uav_conflict_count'],
        'battery_feasible': True,
    }


def run_cli(*arguments):
    subprocess.run([sys.executable, '-B', '-X', 'utf8', str(SOURCE / 'run.py'), *map(str, arguments)],
                   cwd=ROOT, check=True)


def check_manifest():
    path = ROOT / 'docs' / 'SOURCE_MANIFEST.json'
    if not path.exists():
        return {'status': 'not_present'}
    entries = load(path)['files']
    for entry in entries:
        file = ROOT / entry['path']
        # Text hashes use LF so Git's Windows line-ending conversion is harmless.
        data = file.read_bytes()
        if entry['normalization'] == 'lf':
            data = data.replace(b'\r\n', b'\n')
        require(hashlib.sha256(data).hexdigest() == entry['sha256'],
                f'Source manifest mismatch: {entry["path"]}')
    return {'status': 'passed', 'files': len(entries)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--smoke', action='store_true', help='Replan and simulate the small scene')
    parser.add_argument('--simulate-baselines', action='store_true', help='Replay both bridge plans in the simulator')
    parser.add_argument('--report', type=Path, help='Write an inspectable JSON verification report')
    args = parser.parse_args()

    source_files = sorted((SOURCE / 'src').rglob('*.py')) + [SOURCE / 'run.py', SOURCE / 'launch.py'] + sorted((ROOT / 'scripts').glob('*.py'))
    for path in source_files:
        compile(path.read_text(encoding='utf-8'), str(path.relative_to(ROOT)), 'exec')
    scenes = [check_scene(name) for name in ('bridge', 'bridge_stl_20', 'other_scenes')]

    upstream = load(ROOT / '数据集' / 'bridge_stl_20' / 'source_manifest.json')
    for filename, field in (('bridge.stl', 'source_stl_sha256'),
                            ('bridge_mission_frame.stl', 'mission_frame_stl_sha256')):
        path = ROOT / '数据集' / 'bridge_stl_20' / filename
        require(hashlib.sha256(path.read_bytes()).hexdigest() == upstream[field], f'STL hash mismatch: {filename}')

    report = {'status': 'passed', 'python': platform.python_version(), 'platform': platform.system(),
              'syntax_files': len(source_files), 'scenes': scenes, 'stl_source_hashes': 'passed',
              'source_manifest': check_manifest(), 'fresh_small_planning_and_simulation': None,
              'bridge_simulation_replays': []}
    if args.smoke:
        folder = SOURCE / 'outputs' / 'release-check' / 'small'
        run_cli('plan', ROOT / '数据集' / 'other_scenes' / 'mission.json', '--output', folder,
                '--snapshot', folder / 'preview.png')
        run_cli('simulate', folder / 'mission.json', '--plan', folder / 'plan.json', '--output', folder)
        report['fresh_small_planning_and_simulation'] = check_scene('other_scenes', folder)
    if args.simulate_baselines:
        for name in ('bridge', 'bridge_stl_20'):
            preset = ROOT / '数据集' / '运行结果' / name
            folder = SOURCE / 'outputs' / 'release-check' / name
            run_cli('simulate', preset / 'mission.json', '--plan', preset / 'plan.json', '--output', folder)
            result = load(folder / 'online_simulation.json')
            require(result['feasible'] and result['collision_count'] == result['inter_uav_conflict_count'] == 0,
                    f'{name}: simulation replay rejected')
            require(result['coverage_count'] == len(load(preset / 'mission.json')['tasks']),
                    f'{name}: replay visit count mismatch')
            report['bridge_simulation_replays'].append({
                'scene': name, 'feasible': result['feasible'], 'coverage_count': result['coverage_count'],
                'collision_count': result['collision_count'], 'inter_uav_conflict_count': result['inter_uav_conflict_count'],
                'completion_time_s': result['completion_time_s']})
    if args.report:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    try:
        main()
    except (ValueError, KeyError, OSError, subprocess.CalledProcessError) as error:
        print(f'CHECK FAILED: {error}', file=sys.stderr)
        raise SystemExit(1)
