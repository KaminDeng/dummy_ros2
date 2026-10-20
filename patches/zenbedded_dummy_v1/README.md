# Zenbedded Dummy v1 补丁记录

## 新手阅读说明

该目录保留“为何没有上游补丁”的来源说明，不应为了出现补丁文件而复制上游实现。
PIC表示把锁定源以位置无关方式编译进插件，解决共享库装载约束；不表示换了wire或绕过授权。
源码SHA、OSAL/zenoh二进制SHA和版本锁需分别一致，单纯加源码注释也会影响完整原文SHA。
本页v1/D08说明属于对应发布阶段；当前v2入口和部署流程读 [主 README](../../README.md)。

本次 ROS 发布不需要修改上游 Zenbedded 源码。

生产 adapter 将 DPSDK 中未修改的传输源码以 PIC 方式编译，并链接已锁定的 OSAL
与 zenoh-c 静态库。`src/dummy_ros_control/cmake/verify_sdk_inputs.cmake` 强制校验
这些源码和二进制的 SHA-256；任何不一致都会使 CMake 配置失败。ROS 侧生命周期、
freshness、授权与有界 read/write 行为均实现在
`dummy_ros_control/GuardedZenbeddedSystem` 和 `DeviceTransport` 中，而不是维护一份
Zenbedded 私有分叉。

实际消费的 DPSDK 提交记录在 `docs/dummy/versions.lock`。R07 唯一需要的上游增量是
D08 测试 peer 的显式 `--drop-feedback-axis` SIL 故障注入参数；它不修改 Zenbedded、
wire-v1、RobotCore 生产行为或实体固件。
