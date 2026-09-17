# Dummy ROS 任务状态

| 任务 | 状态 | 证据 | 剩余条件 |
| --- | --- | --- | --- |
| R00 | PASS | 用例 AUD01-AUD03 与原生 Gazebo 身份探针均通过，共 4/4。 | 证据：`build_dummy/reports/R00.json`。 |
| R01 | PASS_WITH_LIMITATION | wire-v2/50:1 镜像、真实 FK 参考链、有限多圈与网格 AABB 映射探针通过，共 7/7。 | 实体网格/坐标系标定与碰撞安全尚未实测。 |
| R02 | PASS_WITH_LIMITATION | 原生 Gazebo 身份、轨迹反馈、六轴稳定性、时钟暂停/恢复和已加载 JTC 探针通过，共 6/6。 | 证据：`build_dummy/reports/R02.json`；仿真参数不代表实体测量值。 |
| R03 | PASS | wire-v2 codec/RPC/调用方所有权快照探针通过，共 7/7。 | 证据：`build_dummy/reports/R03.json`。 |
| R04 | PASS | 用例 10/10；生命周期错误通过 worker 实际发送有界 STOP，不再以 blocked 计数替代。 | 证据：`build_dummy/reports/R04.json`；授权正路径由 R06/R07 记录。 |
| R05 | PASS_WITH_LIMITATION | 原生轨迹反馈、场景一致性、碰撞拒绝、笛卡尔失败与取消探针通过，共 7/7。 | 证据：`build_dummy/reports/R05.json`；实体碰撞安全仍未验证。 |
| R06 | PASS | 23/23；wire-v2 bootstrap、compiled-policy permit、单次 permit 续权、两次成功、取消与异常终止均通过真实 D08 peer。 | 证据：`build_dummy/reports/R06.json`。 |
| R07 | PASS | 8/8；J1/六轴生产 launch 使用 D08 peer 与 guarded bridge，SEN06 缺轴故障不自动降级。 | 证据：`build_dummy/reports/R07.json`。 |
| R08 | PASS | REL01 校验 DPSDK commit、wire-v2/hash、peer 构建时效与本轮 R06/R07 证据，1/1 通过。 | 证据：`build_dummy/reports/R08.json`。 |
| 实体硬件/J1 | NOT_RUN | 全程使用 `--hardware none`；未探测、下载、复位或执行实体运动。 | 需要另行执行人工门禁流程。 |

R00 详细报告位于 `docs/dummy/task-reports/R00.json`。构建证据位于
`build_dummy/reports/`，不得将其表述为实体硬件证据。
