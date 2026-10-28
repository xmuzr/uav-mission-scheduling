# 五跨 STL 梁桥数据

模型来源 CSUCRYE/bridge-inspection-ai 的 assets/bridge.stl，MIT 许可证见本目录 LICENSE。
模型为 204 个三角面、17 个闭合构件，五跨各 18 米，总包围盒为 95×12×12 米。
source_manifest.json 记录原始文件摘要和坐标变换；bridge.stl 为原始模型，bridge_mission_frame.stl 为米制任务坐标模型。
mission.json 内嵌模型顶点和面片，规划无需读取外部绝对路径。
viewpoints.json 和 viewpoints.csv 记录本项目配置的 20 个仿真视点及观察目标。


从完整代码目录运行：
项目命令.cmd plan ../数据集/bridge_stl_20/mission.json --output outputs/stl_new
