# 验证说明

发布整理日期：2026-10-28。项目版本：1.0.0。

## 已执行检查

- 两个桥梁场景各为 3 架无人机、20 个任务点；小场景为 2 架、4 个任务点。
- 输入模型、在线参数和障碍物加载通过。
- 保存规划的输入指纹与任务文件一致；任务恰好分配一次，保存的规划约束检查全部通过。
- 保存的三个仿真均完成全部任务点访问，障碍碰撞和机间冲突计数均为 0，续航检查通过。
- 使用 Windows 便携 Python 3.13.17 从头运行小场景规划、快照和仿真；任务完成 4/4、碰撞 0、机间冲突 0。
- 使用同一运行环境重新执行两组桥梁保存规划的在线仿真，分别完成 20/20 个任务点访问，碰撞和机间冲突计数均为 0。
- 原始 STL 与任务坐标 STL 的 SHA-256 和既有来源清单一致。
- 对项目 Python 文件执行语法编译检查，明细和结果见 [便携环境验证记录](validation.json)。
- 新建独立 Python 3.13.5 虚拟环境，通过 `pip install -e ./完整代码` 安装源码及依赖，再执行小场景规划、PNG 和在线仿真，全部通过；49 个 Python 文件语法及 65 个源码/输入摘要检查通过。见 [源码安装验证记录](validation-source-install.json)。

重新执行桥梁在线仿真使用已有规划，没有重新求解两组桥梁的离线分配和 A* 航段。保存的桥梁规划耗时是历史运行记录，不是本次整理的实测耗时。

## 运行校验

在仓库根目录、依赖安装完成的 Python 环境中执行：

```bash
python scripts/check_repository.py --smoke
```

需要同时重跑两组桥梁的在线仿真：

```bash
python scripts/check_repository.py --smoke --simulate-baselines --report outputs/verification.json
```

脚本检查真实输入、输出和输入指纹，并调用项目原有 CLI 完成小场景端到端执行。结果写到 `完整代码/outputs/release-check/`，不覆盖保存基线。Windows 便携包也可将上述 `python` 替换为 `完整代码/runtime/python.exe`。

`docs/SOURCE_MANIFEST.json` 记录源码、入口、配置和原始场景输入摘要，文本按 LF 归一化，二进制 STL 按原始字节计算。修改这些文件后，应先检查变更，再运行：

```bash
python scripts/update_manifest.py
python scripts/check_repository.py --smoke
```

GitHub Actions 使用 Python 3.13 安装源码包并执行相同检查，其状态以仓库 Actions 页面为准。

## 结果口径

`plan.json` 的 `makespan_after_scheduling_s` 表示包含地面错峰等待的计划完成时间。`online_simulation.json` 的 `completion_time_s` 包含离散在线执行、避障和等待，二者不同属于正常现象。

两组桥梁保存结果均为 `pci_greedy` 启发式分配，`optimality_proven=false`。原交付目录 `实验结果图/` 中存在“精确最优”及不同总航程标注的历史图片，不能以其替代当前基线 JSON；首页主图使用与当前基线一致的 `preview.png`。

当前场景的任务点访问不等于影像覆盖；两个桥梁输入的额外在线球体障碍列表为空，不能据此宣称已验证运动障碍场景。未进行真实无人机飞行或飞控链路测试。

## 发布内容

源码仓库包含项目代码、场景、预置结果、PNG 展示图、来源记录及 Markdown 文档。Windows 便携运行时仅放入 Release 下载包。Word 文档、本机缓存、生成的检查输出、下载工具及凭据均不进入源码仓库和便携下载包。
