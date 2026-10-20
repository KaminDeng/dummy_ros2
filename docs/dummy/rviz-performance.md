# 完整 STEP 模型 RViz 显示性能

## 当前状态：已恢复原始视觉网格

2026-10-21，新内核生效后，`glxinfo -B` 显示 Intel Arc（MTL）、
`direct rendering: Yes` 和 `Accelerated: yes`。六个彩色视觉 OBJ 已从
优化前的备份逐字节恢复，总面数恢复为 722241；SHA256 与历史报告中各文件的
`original_sha256` 一致。当前使用原始网格由报告的 `active_visual_variant: original`
和 `restored_on` 标记，下面的减面统计仍作为历史记录保留。

RViz 帧率上限保持原有的 30FPS，实际和目标虚影均启用彩色视觉。
当时未保留软件渲染、线程数或帧计数探针覆盖，本次不需要移除启动环境设置。
碰撞网格、材质、关节变换和 SRDF 均保持恢复前的字节内容。
已安装面板描述资源通过符号链接引用源码；下次启动 RViz 会加载恢复后的网格。

## 先判断视觉瓶颈，不改碰撞预算

FPS是特定窗口、视角、负载与渲染器条件下的采样结果；配置30FPS只是上限。
视觉OBJ减面与碰撞STL属于不同资源，不能为了显示帧率降低碰撞包络。
0.147mm是离散双向采样误差，不是所有连续表面的严格Hausdorff界。
工具会写候选资源，使用独立输出目录并从保存原件出发，避免反复简化累积误差。
本页旧失败断言与实际报告保留；重启RViz不等于重启MoveIt或使其重载碰撞资源。

2026-10-20，用户反馈约5FPS。排查发现真实 OpenGL 为
`llvmpipe (LLVM 20.1.2)`，Xwayland 报 `screen 0 does not appear to be DRI3 capable`。
配置的 Frame Rate 已为30；完整彩色模型原为722241三角面，实际与半透明目标
同时绘制时约144万面，RViz消耗约4至5个CPU核心。没有改动显卡权限或桌面会话。

仅优化六个 `full_step_*-colors.obj` 的视觉网格，按原材质分组独立保拓扑/边界
减面。超过0.15mm双向采样误差的组保留原件，总面数降至553751，减少23.3%。
全部原/新顶点与三角形中心的双向采样最大误差0.147mm；这是采样验证，
不是连续曲面的严格误差上界。MTL、碰撞STL、URDF关节变换和SRDF均未修改。
逐组结果、视觉身份与碰撞身份在
`src/dummy-ros2_description/meshes/full_step_visual_optimization.json`。

在相同RViz配置/双模型、30FPS上限、不执行实机运动的条件下，用临时
`glXSwapBuffers` 计数器每5秒统计：原模型稳定中位数约13.75FPS，优化后
约15.44至16.46FPS。原先5FPS未稳定复现，不承诺稳定30FPS；视角、窗口大小和
系统负载会影响结果。限制软件渲染到4个线程没有改善，未保留该设置。
最终只重启RViz，保留原真机面板、MoveIt；临时计数探针与线程覆盖已移除。

可复用的离线工具为 `scripts/dummy/optimize_step_visuals.py`，依赖
`numpy trimesh vtk`，必须生成到独立目录，检查候选后再部署：

```bash
python scripts/dummy/optimize_step_visuals.py /path/to/original_meshes /path/to/candidate_meshes
```

不要对已简化资源反复累积简化；使用保存的原始彩色网格。工具只输出OBJ、MTL和
验证JSON，不读写碰撞资源，不连接ROS/硬件。工具保留材质/源文件的回归通过，
完整模型资源、减速器、目标标记等针对性回归通过。扩大运行模型历史测试时，
仍有两项旧断言不兼容完整STEP迁移（仅支持7个链接、全部碰撞为box/STL）；
本轮没有修改这些断言或碰撞模型。

本机完整证据位于相邻DPSDK的 `build_dummy/reports/rviz-fps-20261020/`，
包括原始视觉备份、逐材质误差、帧计数日志与 `fps-comparison.json`。
