# 源码索引

以下路径相对于 `完整代码/src/uav_cooperation/`。

| 模块 | 入口或关键文件 | 职责 |
| --- | --- | --- |
| 应用 | `application/cli.py` | validate、plan、simulate、view 子命令与退出码 |
| 规划流水线 | `application/pipeline.py` | 输入指纹、阶段编排、最终约束复核 |
| 输入 | `models/request.py` | 数据类、字段验证和默认参数 |
| 几何 | `models/geometry.py` | 场景、障碍物和空间几何 |
| 网格 | `environment/voxel_grid.py` | 三维占据网格、安全膨胀 |
| 碰撞 | `environment/collision.py` | 点和线段碰撞检查 |
| 三角网格 | `environment/mesh.py` | STL 读取、闭合构件验证、网格距离与相交 |
| 三维规划 | `path_planning/astar/planner.py`、`search.py` | 端点接入、Weighted A* 搜索 |
| 航段复用 | `path_planning/path_store.py` | 航段缓存与反向读取 |
| 路径后处理 | `path_planning/postprocessing/` | 路径简化与复核 |
| 分配 | `task_allocation/solver.py` | OR-Tools Routing、PCI + 贪心下降 / GLS |
| 分配节点 | `task_allocation/nodes.py` | 任务与起降节点身份 |
| 分配矩阵 | `task_allocation/cost_matrix/strategies.py` | 欧氏与 A* 候选点对 |
| 路线指标 | `task_allocation/metrics.py` | 航程、速度、驻留及任务时间 |
| 续航 | `constraints/battery.py` | 可用飞行时间检查 |
| 时间轨迹 | `scheduling/trajectory.py`、`path_data.py` | 规划路径到带时间轨迹 |
| 冲突与错峰 | `scheduling/conflicts.py`、`resolution.py` | 机间冲突检测、起飞延迟搜索 |
| 局部避障 | `realtime_avoidance/api.py`、`local_avoidance.py` | 单周期在线避障接口与策略 |
| 在线障碍 | `realtime_avoidance/obstacles.py`、`config.py` | 球体障碍及在线参数 |
| 仿真 | `simulation/api.py`、`online.py` | 规划匹配、离散执行、轨迹与统计 |
| 导出 | `io/exports.py` | JSON、CSV 文件 |
| 可视化 | `visualization/viewer.py` | 三维航线、时间滑块、播放与 PNG |

项目级入口：

- `完整代码/run.py`：从任意工作目录调用 CLI，插入相对源码路径。
- `完整代码/launch.py`：内置场景选择、重算、浏览器回放。
- `完整代码/启动项目.cmd`：Windows 菜单，优先使用便携运行时，否则使用已安装依赖的 `python`。
- `完整代码/项目命令.cmd`：Windows CLI 包装器。
- `scripts/check_repository.py`：语法、输入、保存结果、输入指纹与小场景端到端检查。
