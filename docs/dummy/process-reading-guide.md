# 模型转换与验证资料导航

从 CAD 到 ROS 的装配转换、显示、碰撞与物理执行分别验证。
[source-map](process-source-map.json)只索引下列三个公开工程文档的 SHA-256、行数与用途；
文档摘要不是模型输入身份、设备许可或新的发布锁。

| 阅读入口 | 内容与边界 |
| --- | --- |
| [STEP 转换流程](model-conversion/README.md) | 输入来源、实例运动归属、局部变换、米/毫米与部署核对；离线预览不授权设备动作 |
| [CAD 导入与碰撞几何](model-conversion/cad-import.md) | XCAF/颜色、OBJ/MTL、支撑包络、SRDF、视觉采样与间隙；不把采样误差当连续界 |
| [Dummy-B v2 装配示例](model-conversion/assembly-example.md) | 源 STEP SHA、180 实例、安装差异与历史状态验证；旧 REST 误报需按后续修复理解 |

现行资源和专题说明：

- [完整模型与安装记录](full-step-model.md)：视觉、碰撞及模型消费者的一致性。
- [逐文件模型资源](mesh-reading-guide.md)：当前引用、历史保留资源及严格 JSON 的单位和身份。
- [REST 分区包络修复](rest-planning.md)：凹支架凸包误报、几何覆盖及双向只读规划。
- [近端与腕部最小间隙](collision-clearance.md)：指定部件对的至少 5mm 规划保护。
- [地面坐标](ground-origin.md)：根坐标统一平移，区别于机械零位修改。
- [视觉性能](rviz-performance.md)：材质减面、渲染条件与离散采样误差。

模型修改后，磁盘资源更新不等于运行中的 MoveIt 已重载碰撞结构。
服务有效、路径采样通过、设备执行和机械停止属于不同证据层；不能用其中一个阶段替代其他阶段。
实际操作入口见[主 README](../../README.md)与[运行手册](ros-runbook.md)。
MoveIt 多实例、圆柱场景及退出步骤在运行手册中说明，关闭规划器不等于设备 Stop/Disable。
