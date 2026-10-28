"""Portable launcher. All built-in paths are relative to this file."""
from pathlib import Path
from datetime import datetime
import argparse
import os
import sys

ROOT = Path(__file__).resolve().parent
DATA = ROOT.parent / '数据集'
SCENES = {'bridge': 'bridge', 'stl': 'bridge_stl_20', 'small': 'other_scenes'}
sys.path.insert(0, str(ROOT / 'src'))


def execute(scene, action, output=None, mission=None):
    # Keep matplotlib config/font caches inside this relocated project.
    os.environ['MPLCONFIGDIR'] = str(ROOT / 'outputs' / '.matplotlib')
    os.environ['MPLBACKEND'] = 'Agg'
    from uav_cooperation.application.cli import main as run
    source = Path(mission).expanduser().resolve() if mission else DATA / SCENES[scene] / 'mission.json'
    if action == 'view' and not mission:
        preset = DATA / '运行结果' / SCENES[scene]
        if not (preset / 'plan.json').is_file():
            raise FileNotFoundError('未找到预置规划结果，请先运行对应场景的重新规划。')
        import matplotlib
        matplotlib.use('WebAgg')
        matplotlib.rcParams.update({'webagg.address': '127.0.0.1', 'webagg.port': 8988,
                                   'webagg.port_retries': 50, 'webagg.open_in_browser': True})
        print('将打开本机浏览器展示三维路线。可旋转、拖动时间滑块和播放。', flush=True)
        print('关闭浏览器页面后，在此窗口按 Ctrl+C 结束回放。', flush=True)
        return run(['view', str(preset / 'mission.json'), '--plan', str(preset / 'plan.json')])
    if action == 'view':
        raise ValueError('自定义场景请先 plan，再使用项目命令.cmd view 与 --plan 指定结果。')
    destination = Path(output).expanduser().resolve() if output else (
        ROOT / 'outputs' / f'{scene}_{datetime.now():%Y%m%d_%H%M%S_%f}')
    if action == 'validate':
        return run(['validate', str(source), '--output', str(destination)])
    print('正在规划，完成后显示结果目录。桥梁场景需要计算三维航段，请等待。', flush=True)
    status = run(['plan', str(source), '--output', str(destination),
                  '--snapshot', str(destination / 'preview.png')])
    if status:
        return status
    if action == 'simulate':
        status = run(['simulate', str(destination / 'mission.json'), '--plan',
                      str(destination / 'plan.json'), '--output', str(destination)])
    print('结果目录：' + str(destination), flush=True)
    return status


def main():
    parser = argparse.ArgumentParser(description='无人机任务调度便携启动器')
    parser.add_argument('--scene', choices=tuple(SCENES), default='bridge')
    parser.add_argument('--action', choices=('validate', 'plan', 'simulate', 'view'))
    parser.add_argument('--output', type=Path)
    parser.add_argument('--mission', type=Path)
    args = parser.parse_args()
    if args.action:
        return execute(args.scene, args.action, args.output, args.mission)
    options = {'1': ('bridge', 'view'), '2': ('bridge', 'simulate'),
               '3': ('stl', 'view'), '4': ('stl', 'simulate'),
               '5': ('small', 'simulate'), '6': ('small', 'view')}
    while True:
        print('\n复杂约束下的无人机飞行任务调度建模与仿真')
        print('1  查看双桥三机航线与时间回放')
        print('2  从头运行双桥任务规划和在线仿真')
        print('3  查看 STL 梁桥三机航线与时间回放')
        print('4  从头运行 STL 梁桥任务规划和在线仿真')
        print('5  运行小场景规划和在线仿真')
        print('6  查看小场景航线与时间回放')
        print('0  退出')
        choice = input('请输入编号：').strip()
        if choice == '0':
            return 0
        if choice not in options:
            print('请输入 0 至 6。')
            continue
        try:
            status = execute(*options[choice])
            print('运行完成。' if status == 0 else f'运行未成功，退出码 {status}，请查看输出目录。')
        except KeyboardInterrupt:
            print('\n已结束当前回放或计算。')
            # A browser server owns the process event loop; restart the launcher to open another viewer.
            return 0
        except Exception as error:
            print(f'运行失败：{error}', file=sys.stderr)


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        raise SystemExit(0)
