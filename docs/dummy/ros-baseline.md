# R00 ROS Baseline

## 中文导读：R00 历史环境基线

本文固定旧工作区、提交、依赖版本和源SHA，用于解释R00当时如何构建；不是当前机器安装清单。
`--hardware none`排除实体IO，静态配置测试成功也不等于Gazebo真正启动或控制器已加载。
表中exit=3明确表示运行时未执行，不能把AUD通过合并为全套PASS。
当前仓库名为`dummy_ros2_consolidated`，新手操作请从 [主 README](../../README.md) 的现行入口开始；
不要直接回退仓库、包版本或源码以追随本历史表。旧路径和摘要保留供报告溯源。

## Scope

R00 establishes an isolated ROS source build and test runner. All R00 commands use
`--hardware none`; no serial device, SocketCAN interface, J-Link process, firmware,
or motor controller is opened.

## Roots And Source State

| Root | State |
| --- | --- |
| ROS_WS: `<home>/kamin_work/00-code/dummy_ros2` | Branch `feat/ros-v3-r00-baseline`, based on `0dc69ac6948dfc8ddbd6ee584431770f01c73a9a`. The pre-existing untracked `build/`, `install/`, and `log/` directories were preserved. |
| DPSDK_ROOT: `../dpsdk` | `323e5fa929cb4ac60583bd97ab374f5486f133a3`; pre-existing modified RTOS/Zenoh submodules and `tools/dummy/` are not owned by ROS. |
| DUMMY_ROOT: `<dummy-code-root>/firmware/dummy-ref-core-fw` | `3a9d17464308e1532b3d648a80da6db899cc1f01`; it already reports local ROS/demo changes. It was read only. |

The R00 source sentinels and their SHA-256 values are:

| Source | SHA-256 |
| --- | --- |
| `src/dummy_moveit_config/launch/gz_moveit.launch.py` | `91c1bf77882439eed3905d5592e1fdd89c845672fa04bb393efff58aaf1c75a9` |
| `src/dummy_moveit_config/config/dummy-ros2.gz_sim.xacro` | `ee8d33abebdf0200c09fad425818f3075b4b7cfe84b539d3df4dfb76770b5318` |
| `src/dummy-ros2_description/urdf/dummy-ros2.xacro` | `6ab3e092d3ee1becbc0971062657616d95774d30ebaec0b8a45ae44cae5692bd` |

## Environment

Ubuntu host: Linux `6.8.0-139-generic`, ROS Jazzy commands available, Gazebo `gz`
commands available, and colcon available. Installed package versions include
`ros-jazzy-gz-ros2-control 1.2.20`, `ros-jazzy-moveit 2.12.4`, and
`ros-jazzy-ros2-control 4.48.0`.

The interactive `python3` is a pyenv 3.14 shim without `catkin_pkg`; the system
`/usr/bin/python3` contains the ROS build dependency. `scripts/dummy/build.sh`
therefore pins only this workspace's CMake Python executable to `/usr/bin/python3`.
It does not change Python, ROS, or package-manager global configuration.

## Results

| Command | Exit | Result |
| --- | ---: | --- |
| `python3 -m unittest discover -s scripts/dummy/tests -p test_r00_acceptance.py -v` | 0 | 5 passed |
| `python3 -m unittest src/dummy_moveit_config/test/test_jazzy_simulation_contract.py -v` | 0 | 3 passed; static configuration evidence only |
| `bash scripts/dummy/build.sh` | 0 | 3 packages built in `build_dummy/install_dummy/log_dummy` |
| `python3 scripts/dummy/verify.py --suite R00 --hardware none --report build_dummy/reports/R00.json` | 3 | AUD01-AUD03 pass; ROS01 is NOT_RUN |

`ROS01` is not a Gazebo PASS. Actual headless spawn, controller activation, and
joint-feedback movement are R02 deliverables after C02 `model-v1` is available.
The R00 runner returns 3 so a missing runtime cannot be silently treated as a pass.

## Next Task

R01 is the next ROS task, but it is blocked on the C02 `model-v1` contract bundle
and manifest hash. R02 remains the first task that may claim a real Gazebo runtime
result.
