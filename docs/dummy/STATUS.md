# Dummy ROS 任务状态

| 任务 | 状态 | 证据 | 剩余条件 |
| --- | --- | --- | --- |
| R00 | PASS | 用例 AUD01-AUD03 与原生 Gazebo 身份探针均通过，共 4/4。 | 证据：`build_dummy/reports/R00.json`。 |
| R01 | PASS_WITH_LIMITATION | wire-v2/50:1 镜像、真实 FK 参考链、有限多圈与网格 AABB 映射探针通过，共 7/7。 | 实体网格/坐标系标定与碰撞安全尚未实测。 |
| R02 | PASS_WITH_LIMITATION | 原生 Gazebo 身份、轨迹反馈、六轴稳定性、时钟暂停/恢复和已加载 JTC 探针通过，共 6/6。 | 证据：`build_dummy/reports/R02.json`；仿真参数不代表实体测量值。 |
| R03 | PASS | wire-v2 codec/RPC/调用方所有权快照探针通过，共 7/7。 | 证据：`build_dummy/reports/R03.json`。 |
| R04 | PASS | 用例 10/10；生命周期错误通过 worker 实际发送有界 STOP，不再以 blocked 计数替代。 | 证据：`build_dummy/reports/R04.json`；授权正路径由 R06/R07 记录。 |
| R05 | PASS_WITH_LIMITATION | 原生轨迹反馈、场景一致性、碰撞拒绝、笛卡尔失败与取消探针通过，共 7/7。 | 证据：`build_dummy/reports/R05.json`；实体碰撞安全仍未验证。 |
| R06 | PASS | 23/23；wire-major-v2 bootstrap、PREPARED/COMMITTED、compiled-policy permit、两次成功、取消与异常终止均通过真实 D08 peer。 | contract=`914074d3...b28f8`；证据：`build_dummy/reports/R06.json`。 |
| R07 | PASS | 8/8；J1/六轴生产 launch 使用 v2 D08 peer 与 guarded bridge，SEN06 缺轴故障不自动降级。 | 证据：`build_dummy/reports/R07.json`。 |
| R08 | PASS | REL01 1/1；DPSDK提交锁、wire-v2 bundle、合同clean、peer时效及R06/R07运行报告均通过发布门禁。 | DPSDK=`c121cc26...8a33`；证据：`build_dummy/reports/R08.json`。 |
| 实体硬件/J1（R00-R08） | NOT_RUN | 上述 R00-R08 使用 `--hardware none`；未探测、下载、复位或执行实体运动。 | 需要另行执行人工门禁流程。 |

R00 详细报告位于 `docs/dummy/task-reports/R00.json`。构建证据位于
`build_dummy/reports/`，不得将其表述为实体硬件证据。

## 2026-10-07 P9 ROS 六轴功能捕获

- 本次 [Action](../../build_dummy/reports/ros-p9-multirun-final10-24debQ/action.json) 对交替的不等六轴目标及反向运动请求 10/10 `SUCCEEDED`，`stop_confirmation=CONFIRMED_RELEASED_IDLE`，active/unresolved 均空。[strict joined](../../build_dummy/reports/ros-p9-multirun-final10-24debQ/joined.json) 与 [application_e2e](../../build_dummy/reports/ros-p9-multirun-final10-24debQ/e2e.json) 均覆盖 10 个目标；E2E 为 `ROS_BENCH_E2E_POSITIVE`，`qualification=UNQUALIFIED`、`hardware_qualification=NOT_ASSERTED`。REF 镜像仍为 `c41be34d47efda4e4eb2b9c7f36f9d4ed3d78d29f60dfeb54055c2623bf9eb22`；本次最终 Gate 为 `6efb…`。
- [context](../../build_dummy/reports/ros-p9-multirun-final10-24debQ/context.json) 记录 State receive-minus-produced offset-span lag `259071us`，超过默认 `250000us`；这次正例使用显式 `500000us` 诊断预算，不能据此声明默认预算通过或发布资格。此前 priorhold fix10、refresh fix10、retry smoke10、独立初始化 init3 等失败报告及未确认 Stop 结论保持有效，独立初始化 3/3 仍未通过。
- 新的 50 个不等六轴目标运行正在进行，尚无结果判定；不得以本次 10/10 推断该组通过。
