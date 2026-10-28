# 第三方来源

## STL 梁桥模型

来源为 [CSUCRYE/bridge-inspection-ai](https://github.com/CSUCRYE/bridge-inspection-ai) 的 `assets/bridge.stl`。原始模型、任务坐标模型、来源清单及原始 MIT 许可均保留在 `数据集/bridge_stl_20/`：

- [原始 MIT 许可证](../数据集/bridge_stl_20/LICENSE)
- [source_manifest.json](../数据集/bridge_stl_20/source_manifest.json)：原始 SHA-256、坐标变换、尺度和几何摘要。
- [模型与视点说明](../数据集/bridge_stl_20/README.md)

20 个仿真视点由本项目配置，不能视为上游提供或影像覆盖保证。

## Python 与依赖

源码版通过 PyPI 安装依赖。Windows 便携包附带 Python 3.13.17 嵌入式环境及 NumPy、OR-Tools、Matplotlib、Tornado 等组件；其原有许可证随运行时目录保留。

完整版本及 wheel 来源摘要见 [依赖来源与许可证](../完整代码/依赖来源与许可证.md)。运行时只放入 Release 下载包，不进入源码 Git 历史。

## 项目自有代码

发布时未为项目自有代码另行选择开源许可证，相关权利由权利人保留。第三方文件按其随附许可处理。公开仓库的可见性不改变第三方许可或自动授予项目自有代码的再分发授权。
