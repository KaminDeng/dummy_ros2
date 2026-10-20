# Dummy ROS 任务状态

## 如何判断表中的结果

R00～R08 是不同阶段、不同身份的记录；PASS 只覆盖该行绑定的报告与当时的后端。
`--hardware none` 是仿真/SIL 证据，不能据此推断当前实体设备已使能、已失能或具有机械资格。
后附 P9 是另一批功能捕获，仍明确 UNQUALIFIED；500000 us 诊断预算也不能替代默认250000 us门。
更新状态应同时给出原始报告、镜像/boot/session身份和未覆盖项，不能用新正例追改旧失败。
当前软件入口见 [主 README](../../README.md)，场景故障处理见 [运行手册](ros-runbook.md)。

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

- 固定 REF 镜像 `c41be34d47efda4e4eb2b9c7f36f9d4ed3d78d29f60dfeb54055c2623bf9eb22`，最终 Gate `6efb…`。同一 REF boot epoch `9868814893579086642` 下，四组为互不相同的 ROS session，分别完成 10/50/10/10 个六轴 Action 目标；每组 `SUCCEEDED`，`stop_confirmation=CONFIRMED_RELEASED_IDLE`，active/unresolved 均空，joined-v2 的 release 均为 `CANCELED_AND_RELEASED` 且 `confirmed=true`。下表列出各组原运行产物的相对路径与历史判定；这些产物不随公开仓库发布，路径不表示全新克隆已包含或可直接读取这些文件：

  | 捕获 | Action | 原始数据 | 最终证据 | 保守 State lag |
  | --- | --- | --- | --- | --- |
  | `multirun-final10-24debQ` | `10/10`（`build_dummy/reports/ros-p9-multirun-final10-24debQ/action.json`） | `build_dummy/reports/ros-p9-multirun-final10-24debQ/raw.bin` | `build_dummy/reports/ros-p9-multirun-final10-24debQ/joined-v2.json` / `build_dummy/reports/ros-p9-multirun-final10-24debQ/e2e-v2.json` | `259071us`（`build_dummy/reports/ros-p9-multirun-final10-24debQ/context.json`） |
  | `unequal-final50-IdUnyK` | `50/50`（`build_dummy/reports/ros-p9-unequal-final50-IdUnyK/action.json`） | `build_dummy/reports/ros-p9-unequal-final50-IdUnyK/raw.bin` | `build_dummy/reports/ros-p9-unequal-final50-IdUnyK/joined-v2.json` / `build_dummy/reports/ros-p9-unequal-final50-IdUnyK/e2e-v2.json` | `320737us`（`build_dummy/reports/ros-p9-unequal-final50-IdUnyK/context.json`） |
  | `frozen-init2-tCqfWq` | `10/10`（`build_dummy/reports/ros-p9-frozen-init2-tCqfWq/action.json`） | `build_dummy/reports/ros-p9-frozen-init2-tCqfWq/raw.bin` | `build_dummy/reports/ros-p9-frozen-init2-tCqfWq/joined-v2.json` / `build_dummy/reports/ros-p9-frozen-init2-tCqfWq/e2e-v2.json` | `421267us`（`build_dummy/reports/ros-p9-frozen-init2-tCqfWq/context.json`） |
  | `frozen-init3-rftX2v` | `10/10`（`build_dummy/reports/ros-p9-frozen-init3-rftX2v/action.json`） | `build_dummy/reports/ros-p9-frozen-init3-rftX2v/raw.bin` | `build_dummy/reports/ros-p9-frozen-init3-rftX2v/joined-v2.json` / `build_dummy/reports/ros-p9-frozen-init3-rftX2v/e2e-v2.json` | `303397us`（`build_dummy/reports/ros-p9-frozen-init3-rftX2v/context.json`） |

- 四份最终 `e2e-v2` 均为 `ROS_BENCH_E2E_POSITIVE`，但仍是 `qualification=UNQUALIFIED`、`hardware_qualification=NOT_ASSERTED`。lag 使用 `state_receive_minus_produced_offset_span` 保守口径；正例均设置显式 `500000us` 诊断预算，四个值均超过默认 `250000us`。H、M1、M2、600s 和默认 250ms 门禁均尚未通过，不能据此声明发布资格或实体硬件合格。
- 此前 priorhold fix10、refresh fix10、retry smoke10、早期独立初始化 init3 等失败报告及各自未确认 Stop 的结论保持有效；新捕获不追溯改写旧报告。`frozen-init3-rftX2v` 内有一次本地 malformed CLI 调用，在运动前以 exit 2 退出；随后有效调用的 10/10 正例以上述 `action.json` 和最终证据为准。进程退出或 shutdown 不能替代已确认的 STOP/release 证据；独立初始化 3/3 的发布门禁仍未通过。
