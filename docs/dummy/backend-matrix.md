# Dummy ROS Backend Matrix

## 中文阅读说明

本表比较早期 mock/SIL/real_j1 的轴数、时钟与传输，方便选择正确测试对象。
`mock_6dof` 的时钟来自 Gazebo `/clock`；SIL 的 steady 时钟不能拿来直接减仿真时间。
SIL 使用 localhost 与 VirtualCAN；real_j1 的只读声明不等于已运行实体测试，缺轴也不能自动补成六轴。
当前 Qt 面板的纯软件后端、Gazebo、设备 Action gate 和 Native 是不同入口，完整导航见
[主 README](../../README.md)。表中的 NOT_RUN 和旧证据保留为当时状态，不覆盖后来的独立六轴记录。

| Backend | Axes | Clock | Transport | Startup safety | Evidence |
| --- | --- | --- | --- | --- | --- |
| `mock_6dof` | Joint1-Joint6 | Gazebo `/clock` | Gazebo | No DPSDK session or hardware | R02/R05 runtime |
| `sil_j1` | Joint1 | steady | localhost Zenoh + VirtualCAN | No physical IO | R07 D06/D07 |
| `sil_6dof` | Joint1-Joint6 | steady | localhost Zenoh + VirtualCAN | No physical IO | R07 independent mask=63 peer |
| `real_j1` | Joint1 | steady | explicit Zenoh endpoint | read-only; no Grant/Capture/Enable | NOT_RUN |

The real backend does not expose Joint2-Joint6 as measured state. Missing six-axis
feedback is an error in the six-axis SIL profile, not a five-axis fallback. None of
the launch files performs automatic authorization, reference capture, enable, or
motion.
