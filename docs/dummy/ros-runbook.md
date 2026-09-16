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
GET_OPERATION 终态，返回 `OPERATION_COMPLETE` 后才能继续：

```sh
ros2 service call /device_action_gate/select_control_mode std_srvs/srv/Trigger
ros2 service call /device_action_gate/grant_session std_srvs/srv/Trigger
ros2 service call /device_action_gate/capture_bench_reference std_srvs/srv/Trigger
ros2 service call /device_action_gate/set_bench_permit std_srvs/srv/Trigger
ros2 service call /device_action_gate/enable_hold std_srvs/srv/Trigger
```

`/device_action_gate/enable` 保留为显式操作员便利入口，按顺序执行经过验证的
Select/Grant/Capture/Permit/Enable bootstrap。launch、read/write 与硬件生命周期
回调均不会调用该服务。

## 实体 J1 只读声明

```sh
ros2 launch dummy_bringup real_j1.launch.py \
  device_id:=<32-hex-id> endpoint:=<explicit-zenoh-endpoint> \
  serial:=<board-serial> profile:=real_j1 read_only:=true
```

该命令只声明显式只读配置。J1 实体后端状态仍为 `NOT_RUN`，不得由此推断连接、
reference、drive enable 或运动有效。任何带电动作都必须通过独立 G6 人工门禁流程，
不属于本次 `--hardware none` 发布范围。

## 公共接口

原生仿真 action 为 `/arm_controller/follow_joint_trajectory`；内部 controller action
仅供 gate 使用。设备管理使用 wire-v1 `manage_v1`，command/state 使用设备限定路由
下的 `command_v1`/`state_v1`。发布哈希记录在 `versions.lock`。
