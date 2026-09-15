# Dummy ROS Runbook

## Build and test

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

## Gazebo six-axis

```sh
source /opt/ros/jazzy/setup.zsh
source install_dummy/setup.zsh
ros2 launch dummy_bringup mock_6dof.launch.py
```

This is simulation evidence only. Collision geometry and position-servo dynamics
are approximations. MoveIt 2.12.4 reproducibly exits with signal 11 during Ctrl-C
teardown after successful headless runs; the runtime assertions complete before
shutdown.

## J1 SIL

```sh
source /opt/ros/jazzy/setup.zsh
source install_dummy/setup.zsh
export DPSDK_ROOT=../dpsdk
ros2 launch dummy_bringup sil_j1.launch.py endpoint:=tcp/127.0.0.1:17447
```

SIL uses localhost Zenoh and VirtualCAN. It never opens J-Link, ttyACM0, USB, or
SocketCAN and is not hardware evidence.

## Real J1 read-only declaration

```sh
ros2 launch dummy_bringup real_j1.launch.py \
  device_id:=<32-hex-id> endpoint:=<explicit-zenoh-endpoint> \
  serial:=<board-serial> profile:=real_j1 read_only:=true
```

This command only declares an explicit read-only configuration. The J1 backend is
`NOT_RUN`; do not infer connectivity, reference validity, drive enable, or motion.
Any powered action requires the separate G6 human-gated procedure and is outside
this `--hardware none` release.

## Public interfaces

The native simulation action is `/arm_controller/follow_joint_trajectory`; its
internal controller action is private to the gate. Device management uses the
wire-v1 `manage_v1` service, and command/state use `command_v1`/`state_v1` under
the device-qualified DPSDK route. Release hashes are in `versions.lock`.
