# 当前协议合同

`contracts/`仅保留当前V2正式合同和仍被离线工具引用的候选材料：

| 目录 | 用途 |
| --- | --- |
| `dummy_robot_v2/` | 当前wire 2.5正式接口、codec、布局及模型；运行代码使用此合同。 |
| `dummy_robot_v2_application_hold_no_load_candidate/` | 离线候选材料；保留其原始身份，不代表已经绑定当前正式合同或固件，不可作为实机授权。 |

V1、C03原型及旧2.4完整合同已从当前工作树移除，可通过Git历史查看。仍需解码旧2.4证据的代码和回归，仅引用[内部兼容资源](../compatibility/wire2_4/README.md)，不将其作为当前控制合同。FK测试所需的两份旧数学样本保留在 `src/dummy_moveit_config/test/fixtures/`，不携带旧协议运行入口。

标准ROS消息由ROS依赖包提供；自有设备字段在本目录的 `idl/*.msg`、`idl/*.srv` 描述，`codec/wire_layout.json`记录二进制偏移，`codec/*.hpp`执行编解码。目录名 `idl` 表示接口定义材料，并不要求文件后缀是 `.idl`。合同清理不改变当前正式包或候选包的任何字节及摘要。
