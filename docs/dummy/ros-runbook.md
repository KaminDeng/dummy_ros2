# Dummy ROS 运行手册

## 构建与测试

```sh
cd <home>/kamin_work/00-code/dummy_ros2
source /opt/ros/jazzy/setup.zsh
export DPSDK_ROOT=../dpsdk
colcon --log-base log_dummy build --build-base build_dummy --install-base install_dummy
source install_dummy/setup.zsh
colcon --log-base log_dummy test --build-base build_dummy --install-base install_dummy
colcon --log-base log_dummy test-result --test-result-base build_dummy --all --verbose
python3 scripts/dummy/verify.py --suite R07 --hardware none --report build_dummy/reports/R07.json
python3 scripts/dummy/verify.py --suite R08 --hardware none --report build_dummy/reports/R08.json
```

## Gazebo 六轴仿真

```sh
source /opt/ros/jazzy/setup.zsh
source install_dummy/setup.zsh
ros2 launch dummy_bringup mock_6dof.launch.py
```

该入口只产生仿真证据。碰撞几何与位置伺服动力学均为近似值。无头模式运行
断言成功后，MoveIt 2.12.4 在 Ctrl-C 清理阶段可复现信号 11；这不应被描述为
实体运动或实体碰撞安全证据。

## J1 SIL

```sh
source /opt/ros/jazzy/setup.zsh
source install_dummy/setup.zsh
export DPSDK_ROOT=../dpsdk
ros2 launch dummy_bringup sil_j1.launch.py endpoint:=tcp/127.0.0.1:17447
```

六轴入口使用同一生产 bridge，profile 为 1、configured mask 为 63：

```sh
ros2 launch dummy_bringup sil_6dof.launch.py endpoint:=tcp/127.0.0.1:17446
```

两个 SIL 入口仅使用 localhost Zenoh 与 VirtualCAN，不打开 J-Link、ttyACM0、
USB 或 SocketCAN。启动后 hardware 保持 inactive，并且不会自动发送任何管理变更。

操作员可逐项执行管理操作；每一步都通过 `bootstrap_status` 轮询原 request 的
GET_OPERATION 终态。前四步返回 `OPERATION_COMPLETE` 后才能继续，最后一步必须
确认 `ENABLED_HOLD`：

```sh
ros2 service call /device_action_gate/select_control_mode std_srvs/srv/Trigger
ros2 service call /device_action_gate/capture_bench_reference std_srvs/srv/Trigger
ros2 service call /device_action_gate/grant_session std_srvs/srv/Trigger
ros2 service call /device_action_gate/set_bench_permit std_srvs/srv/Trigger
ros2 service call /device_action_gate/enable_hold std_srvs/srv/Trigger
```

`/device_action_gate/enable` 保留为显式操作员便利入口，按顺序执行经过验证的
Select/Capture/Grant/Permit/Enable bootstrap。launch、read/write 与硬件生命周期
回调均不会调用该服务。

Permit 是单次运动授权。每个成功、取消或失败动作后，会话、reference 与 HOLD
保持，但公开 action 会关闭新目标接收；操作员必须再次调用
`/device_action_gate/set_bench_permit` 并等待 `bootstrap_status` 返回
`OPERATION_COMPLETE`。coordinator 为每次显式授权生成不重复 nonce，不会重放已消费 digest。

## 实体 J1 只读声明

```sh
ros2 launch dummy_bringup real_j1.launch.py \
  device_id:=<32-hex-id> endpoint:=<explicit-zenoh-endpoint> \
  serial:=<board-serial> profile:=real_j1 read_only:=true
```

该命令只声明显式只读配置。J1 实体后端状态仍为 `NOT_RUN`，不得由此推断连接、
reference、drive enable 或运动有效。任何带电动作都必须通过独立 G6 人工门禁流程，
不属于本次 `--hardware none` 发布范围。

## 真实六轴无负载开发证据

真实六轴 profile 固定为 `f405_6dof_bench`、ID 201、mask 63。仓库内的
`design_seed_default_deny` bundle 仍只供离线检查；实际 `NO_LOAD_DEV` 必须使用
冻结的 D 产物路径清单、与其匹配的 candidate bundle 和 ROS limits JSON。activation
文件的 SHA-256 必须从该清单之外独立核对并传入，不能从待验证的清单中自取。
`BENCH_SESSION` 目标是本次参考零位的相对弧度，每轴限于 `[-0.6, 0.6]`；规划速度
上限为约 `0.1508 rad/s`，加速度 `1.0 rad/s²` 仅为 advisory，jerk 未获资格。
这些限值和 `UNQUALIFIED` 身份均不构成机械装配或发布资格。旧的离线预检仍可运行：

```sh
/usr/bin/python3 scripts/dummy/real_six_axis_preflight.py \
  --bundle contracts/dummy_robot_v2 \
  --endpoint 'serial//dev/ttyACM1#baudrate=115200' \
  --device-id <设备32位十六进制ID> --client-id <Host32位十六进制ID> \
  --serial <板卡序列号> --report build_dummy/reports/ros-six-axis-preflight.json
```

预检故意以非零状态退出，不能当成在线激活。真实开发会话须先核对清单中的 image、
USB by-id 串口、设备 ID、candidate manifest 与 ROS limits 均属于同一冻结 D 身份；
新 D image 目前尚未完成资格确认。曾用 7d89 产物尝试的 action 因未激活硬件和
controller 而报 `GUARDED_REGISTRATION_FAILED`，不是真实 ROS E2E 通过记录。

以下为获得对应开发门禁后、由操作员在独占终端执行的顺序。将占位路径和哈希替换为
**同一批已核对的绝对路径与外部 SHA-256**；`run_real_6dof_session.py` 持有唯一
launch 进程组并检查 by-id tty 与已有 gate/launch 进程。它运行期间不可同时启动
另一个 `real_6dof.launch.py`、原生 CLI 或其他串口 owner。保持此终端运行，在第二
终端执行后续命令；退出时脚本清理自己的进程组并复查串口占用。

```sh
source /opt/ros/jazzy/setup.zsh
source install_dummy/setup.zsh
mkdir -p /absolute/report-root/session-001
/usr/bin/python3 scripts/dummy/run_real_6dof_session.py \
  --deployment-paths /absolute/frozen/no-load-dev-deployment-paths.json \
  --activation-sha256 <外部核对的activation文件SHA-256> \
  --ros-limits-json /absolute/frozen/no-load-dev-ros-limits.json \
  --raw-evidence-report-root /absolute/report-root/session-001 \
  --raw-evidence-path /absolute/report-root/session-001/raw-evidence.jsonl \
  --goal-timeline-path /absolute/report-root/session-001/goal-timeline.jsonl \
  --startup-mode full
```

两个采集目标必须是该目录下尚不存在的不同绝对路径；每次会话使用新的目录和文件名。
三个采集参数必须同时提供，省略全部参数则关闭采集。脚本在获取串口 owner 前验证路径，
并将两个采集文件交给同一次 launch 的 hardware 与 gate，不另启串口会话。

launch 默认让 hardware 与 `arm_controller`、`joint_state_broadcaster` 保持 inactive，
也不自动授予 enable。第二终端先明确请求 bootstrap，再轮询到成功且消息恰为
`ENABLED_HOLD`；排队成功、`BOOTSTRAP_PENDING` 或单独的 `OPERATION_COMPLETE`
均不足以开始规划。只有确认 HOLD 后才执行以下 `BENCH_SESSION` 调用：

```sh
source /opt/ros/jazzy/setup.zsh
source install_dummy/setup.zsh
ros2 service call /device_action_gate/enable std_srvs/srv/Trigger
ros2 service call /device_action_gate/bootstrap_status std_srvs/srv/Trigger
/usr/bin/python3 scripts/dummy/ros_six_axis_movej.py \
  --reference-frame BENCH_SESSION \
  --target 0.01 -0.02 0.01 -0.01 0.02 -0.03 \
  --activate-controller \
  --activation-sha256 <与launch相同的SHA-256> \
  --device-id <清单中的32位十六进制设备ID> \
  --candidate-manifest /absolute/frozen/candidate-bundle/manifest.json \
  --ros-limits-json /absolute/frozen/no-load-dev-ros-limits.json \
  --report build_dummy/reports/ros-six-axis-action.json
```

CLI 会重新核对在线 gate 身份、HOLD 和 limits，然后显式激活 hardware 与两个
controller；正常退出也会经 gate 执行 `DisableAndRelease`，确认 `RELEASED_IDLE`，
再停用由本次 CLI 激活的 ROS 组件。若报告 `UNCONFIRMED_STOP`、仍有 active/未知
组件，须按未确认停止处理，不能复用该会话。一次运动消耗 permit；后续动作需要新的
显式授权及状态确认。

该报告仅是 `ROS_ACTION_ONLY`，即使状态为 `SUCCEEDED` 也不构成设备运动通过。
必须另行采集 REF run/session/epoch、六轴 CAN group SENT/committed、真实逐轴反馈
和 Hold/Stop 终态，按 goal UUID 关联成原始 E2E 文件，再执行：

```sh
/usr/bin/python3 scripts/dummy/application_e2e.py \
  --evidence <六轴ROS与设备CAN关联原始文件.json> \
  --report build_dummy/reports/ros-six-axis-e2e-check.json
```

校验只检查提供的原始记录内部一致性，不能代替独立采集来源校验、D/H 门禁或机械
装配资格，也不能声称 `RELEASE_APPROVED`。旧 J1 peer、native CLI PASS 与六轴 SIL
不计入真实 ROS/MoveIt E2E；当前新 D image 仍需独立资格确认及真实联动证据。

## 公共接口

原生仿真 action 为 `/arm_controller/follow_joint_trajectory`；内部 controller action
仅供 gate 使用。设备管理使用 wire-v1 `manage_v1`，command/state 使用设备限定路由
下的 `command_v1`/`state_v1`。发布哈希记录在 `versions.lock`。
