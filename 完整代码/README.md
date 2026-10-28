# 复杂约束下的无人机飞行任务调度建模与仿真

GitHub 源码版的安装、成果展示与文档入口见 [仓库首页](../README.md)。源码版不附带 `runtime/`；可先执行 `python -m pip install -e .`，再运行 `python launch.py`。下文的免安装方式适用于 Release 的 Windows 便携包。

## 直接运行

在 Windows 10/11 64 位 x64 电脑上，完整解压交付压缩包，进入本目录，双击 **启动项目.cmd**。无需安装 Python、Conda、MATLAB、编译器或联网安装依赖。

菜单 1 和 3 打开已保存的双桥、STL 梁桥三维航线，支持鼠标旋转、时间滑块及播放。页面由本机 127.0.0.1 提供，不上传任务数据。关闭网页后在命令窗口按 Ctrl+C 结束回放；再次双击可重新打开。

菜单 2 和 4 从输入重新计算任务分配、A* 航段、错峰调度和在线仿真；菜单 5 运行小场景。桥梁规划通常需要等待数分钟，命令窗口会显示航段进度。新结果写入本目录 outputs 下按时间命名的子目录。

请完整解压后启动，保持“完整代码”和“数据集”的同级关系。整个项目可移动至其他盘符或中文、含空格的目录；不需要复制到作者电脑的原位置。项目目录需要写权限。建议解压在本地磁盘，避免过长路径。

## 文件

src/uav_cooperation 为完整算法；便携包中的 runtime 是独立 Python 及运行依赖；run.py 为命令行入口；launch.py 为便携菜单；docs/输入输出说明.md 为输入合同；requirements.txt 锁定交付环境全部依赖版本。数据、STL 模型、视点和预置结果位于同级“数据集”，系统说明见上一级 docs/ARCHITECTURE.md。Word 技术资料仅保留于本地。

## 命令行

在本目录打开终端。PowerShell 运行 cmd 脚本时加 .\ 前缀：

```powershell
.\项目命令.cmd validate ../数据集/bridge/mission.json
.\项目命令.cmd plan ../数据集/bridge/mission.json --output outputs/bridge_new --snapshot outputs/bridge_new/preview.png
.\项目命令.cmd simulate outputs/bridge_new/mission.json --plan outputs/bridge_new/plan.json --output outputs/bridge_new
.\项目命令.cmd view outputs/bridge_new/mission.json --plan outputs/bridge_new/plan.json
.\项目命令.cmd plan ../数据集/bridge_stl_20/mission.json --output outputs/stl_new
```

也可不切换当前目录，用完整路径调用 runtime/python.exe 和 launch.py：

```powershell
.\runtime\python.exe -B -X utf8 launch.py --scene small --action simulate
.\runtime\python.exe -B -X utf8 launch.py --scene bridge --action simulate
.\runtime\python.exe -B -X utf8 launch.py --scene stl --action view
.\runtime\python.exe -B -X utf8 launch.py --action simulate --mission 自定义任务.json
```

自定义 --mission 和 --output 的相对路径按调用者当前目录解析；内置场景路径始终相对 launch.py。默认分配采用 PCI 加贪心下降，--allocation-strategy pci_gls 可切换为 GLS；--strategy euclidean/hybrid/astar 控制分配距离矩阵，最终实际航段均使用 A*。

退出码 0 表示成功，2 表示输入、规划或仿真不满足条件。检查本次退出码及输出的 status/feasible；失败时参考 error.json，不要把旧 plan.json 当成本次结果。修改输入后必须重新规划，仿真与回放会核对输入指纹。

## 集成

包内环境可直接导入 src 中的 uav_cooperation。使用其他操作系统或公司已有 Python 时，需另建 Python 3.11 以上环境并安装 requirements.txt 中的依赖；附带的 runtime 仅用于 Windows x64。

```python
from uav_cooperation import plan_mission
from uav_cooperation.simulation.api import simulate_mission
from uav_cooperation.io.exports import export_plan

plan = plan_mission(data)  # data 为任务 JSON 解析得到的字典
if not plan['feasible']:
    raise RuntimeError('本次任务未满足规划约束')
export_plan('outputs/current', data, plan)
simulation = simulate_mission(data, plan)
```

包内环境采用隔离路径，不读取用户全局 site-packages、PYTHONPATH 或 Conda 环境。runtime 不是需要安装的虚拟环境，不要修改 python313._pth 中的相对目录。

## 功能范围

支持不同起降点、速度和可用飞行时间的多机任务分配；长方体、圆柱和闭合三角网格避障；三维 Weighted A*；错峰起飞；静止/运动球体与邻机的在线避障及数值仿真。时间单位为秒，位置为本地米制直角坐标。

该交付为算法与仿真项目，不包含飞控通信、MAVLink、地面站硬件驱动或真实飞行控制。任务点访问完成不等于构件影像全覆盖；续航由可用秒数减预留秒数表示；限时启发式分配不承诺全局最优。完整字段和实现边界见上一级 docs/ARCHITECTURE.md 及 docs/输入输出说明.md。
