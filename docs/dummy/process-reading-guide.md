# 模型转换流程与历史设计阅读指南

本页补充五份流程/设计资料的中文阅读导航，原件全部保留原字节。
[source-map](process-source-map.json)给出逐文件SHA、行数和路径，用于核对所读版本；
摘要不是新发布锁、设备身份或运行授权。

## STEP 模型流程的三层资料

| 原件 | 负责解释什么 | 初学者重点 |
| --- | --- | --- |
| [SKILL.md](../../.agents/skills/step-to-ros-model/SKILL.md) | 从装配来源核对、运动归属、局部坐标、附加件安装到SRDF/隔离预览的工作顺序 | 转换成功、显示正确、碰撞有效、实机通过分别判定；此流程不自动授权USB控制或运动。 |
| [conversion-details.md](../../.agents/skills/step-to-ros-model/references/conversion-details.md) | XCAF实例/颜色、OBJ材质、保守包络、碰撞豁免、性能和部署核对 | 毫米转米只做一次，实例变换只累乘一次，颜色空间转换不重复，源面丢失不能忽略。 |
| [dummy-b-v2-case.md](../../.agents/skills/step-to-ros-model/references/dummy-b-v2-case.md) | Dummy-B特定STEP身份、180实例、减速器安装点、原测量与保留问题 | 具体SHA/尺寸/孔距/耗时只属于该案例；旧Rest碰撞结论需结合后续修复理解。 |

`T_link_mesh = inverse(T_world_link(q_ref)) × T_world_part` 是在参考姿态下把CAD世界坐标
变为连杆局部坐标。网格已烘焙变换时不能再叠加一次URDF origin。装配中名为J6的电机壳体
可能固定在上游连杆上；运动归属根据真实固定关系确定，不能按部件名字猜测。

STEP中的同一个零件定义可以出现多个实例。读取实例路径、Location和父级变换后，每个实例
仍需保留；不能为去重把多个螺丝删成一个。Solid/Face等不同层的颜色要按覆盖和继承关系解析。
STL不保存材质颜色，OBJ还需随包安装其MTL；统一URDF颜色可能覆盖源材质显示。

视觉网格可以按材质组减面；碰撞网格应保守覆盖源几何。支持平面交集
`d_i·x <= max(d_i·p)+margin` 的外包络会覆盖输入点，前提包括方向覆盖三维、交集有界和法向归一。
162方向、0.3mm包络、5mm间隙保护属于各自用途，不能把网格误差补偿当成任意硬件安全间隙。
几何采样误差也不是所有连续曲面的严格误差上界。

SRDF的AllowedCollisionMatrix按连杆对豁免，不只豁免某个孔或接触面。新增固定子连杆可缩小
豁免范围，却不新增可动自由度；源文件修改后还需核运行中的ACM。整件凹支架凸包可能填实
真实让位槽，解决办法是审查包络，不是批量增加Never豁免。

本案例的旧Rest接触现已有后续[REST细化与双向规划修复](rest-planning.md)。
早期SKILL案例保留“尚需解释”的时点，不能据其历史措辞撤销修复或放行当时的坏包络。
当前完整装配、地面原点与近端保护分别读[完整模型](full-step-model.md)、
[地面坐标](ground-origin.md)、[最小间隙](collision-clearance.md)。
运行中的MoveIt/实际TF/目标TF必须加载同一资源；重启RViz本身不会重载MoveIt的内存碰撞结构。

## 2026-09-15 的 v3 设计与计划

| 原件 | 作用 | 阅读边界 |
| --- | --- | --- |
| [修复设计](../superpowers/specs/2026-09-15-v3-conformance-repair-design.md) | 区分Gazebo与设备SIL，定义case证据、R06生命周期、R07后端和R08发布依赖 | wire-v1/model-v1、NOT_RUN和阶段状态是当时上下文，不能覆盖后续v2/真实开发记录。 |
| [实施计划](../superpowers/plans/2026-09-15-v3-conformance-repair.md) | 把设计拆为5项任务、文件清单、RED/修复/验证与提交步骤 | 未勾复选框不是当前实现缺失的自动证明；先查实际代码与绑定报告，不重复开发或重放设备命令。 |

证据runner应逐case产生独立观察/trace/hash，再据实际状态汇总；不能把一个成功结果盖给
所有fixture。缺少运行依赖的NOT_RUN必须与PASS区分。静态配置、编译、真实Gazebo、
本地RobotCore/VirtualCAN、实际设备分别属于不同验证层。

设备Action链由公开gate、异步管理客户端、命令producer和内部JTC注册协作。Begin回执、
GET_OPERATION终态、首样本、冻结末样本、Complete、Hold确认和外层Action成功是不同阶段。
取消/异常应先让旧producer代次失效，防迟到回调恢复旧目标；注册UUID/digest也不代替设备permit。
相关原理可以沿用，但具体操作名、路由和合同必须使用当前版本，不能把历史wire-v1清单当v2调用入口。

历史设计/计划不在这里重新执行或改勾选状态，原资料中的命令和硬件限制也按其阶段理解。
新手实际操作从[主README](../../README.md)和[运行手册](ros-runbook.md)开始；
MoveIt多实例、圆柱场景及退出步骤在运行手册专节，关闭规划器不等于机械停车或失能。
