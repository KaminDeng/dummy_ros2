# Dummy ROS 任务状态

| 任务 | 状态 | 证据 | 剩余条件 |
| --- | --- | --- | --- |
| R00 | PASS | 用例 AUD01-AUD03 与原生 Gazebo 身份探针均通过，共 4/4。 | 证据：`build_dummy/reports/R00-repair.json`。 |
| R01 | PASS_WITH_LIMITATION | 模型包、真实 FK 参考链、有限多圈与网格 AABB 映射探针通过，共 7/7。 | 实体网格/坐标系标定与碰撞安全尚未实测。 |
| R02 | PASS_WITH_LIMITATION | 原生 Gazebo 身份、轨迹反馈、六轴稳定性、时钟暂停/恢复和已加载 JTC 探针通过，共 6/6。 | MoveIt 在 Ctrl-C 清理时仍以 -11 退出；仿真参数不代表实体测量值。 |
| R03 | PASS | codec/RPC/调用方所有权快照探针通过，共 7/7；C04/C05 使用 ROS 侧本地扩展，未声称修改上游源码。 | 设备模式跨进程旁路闭合由 R06 验证。 |
| R04 | PASS | 用例 10/10、生产传输 CTest 15/15；真实 D06 负路径、合成组件正路径、身份失效、源码锁与无分配 read/write 分开记录。 | 授权后的真实设备正路径由 R06/R07 记录。 |
| R05 | PASS_WITH_LIMITATION | 原生轨迹反馈、场景一致性、碰撞拒绝、笛卡尔失败与取消探针通过，共 7/7。 | 实体碰撞安全与 MoveIt 清理问题仍未验证。 |
| R06 | PASS | 23/23；真实 D08 peer 验证显式 bootstrap、两次成功、取消、异常终止、operation_id、epoch、ticket 与首个 HOLD barrier。 | 证据：`build_dummy/reports/R06.json`。 |
| R07 | PASS | 8/8；J1/六轴生产 launch 使用 D08 peer 与 guarded bridge。SEN06 在同一六轴 ROS 传输进程观察到 J3 超时（`raw_valid_mask=59`、`FAULT_LATCHED`），未自动降级。 | 证据：`build_dummy/reports/R07.json`；六轴结论未使用 J1 结果替代。 |
| R08 | PASS | REL01 发布锁与证据分层通过。 | 证据：`build_dummy/reports/R08.json`。 |
| 实体硬件/J1 | NOT_RUN | 全程使用 `--hardware none`；未探测、下载、复位或执行实体运动。 | 需要另行执行人工门禁流程。 |

R00 详细报告位于 `docs/dummy/task-reports/R00.json`。构建证据位于
`build_dummy/reports/`，不得将其表述为实体硬件证据。
