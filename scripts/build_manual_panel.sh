#!/usr/bin/env bash
# @file build_manual_panel.sh
# @brief 用系统Jazzy构建手动面板的描述/MoveIt/bringup三个包。
# @details 解析本脚本位置作为工作区根，清除历史underlay路径，再加载系统ROS。
# 输出到独立build/install/log_panel_guide；不运行面板或连接设备。
# 只构建手动面板所需的三个本地包；安装区可独立用于 real / simulator。
# 命令或管道失败即退出；先source发行版环境再开启-u，避免其初始化脚本读取未定义变量时报错。
set -eo pipefail
PANEL_WORKSPACE=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
cd "$PANEL_WORKSPACE"
# 避免误用历史 ROS 工作区作为 underlay；只依赖系统 Jazzy。
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH
source /opt/ros/jazzy/setup.bash
set -u
# 只选这三个源码包且关闭其测试，symlink-install便于读取更新资源；
# 所有额外参数按原argv边界转发。此脚本不构建dummy_ros_control/motion_client或导出DPSDK SDK。
colcon --log-base log_panel_guide build \
  --base-paths src/dummy-ros2_description src/dummy_moveit_config src/dummy_bringup \
  --build-base build_panel_guide --install-base install_panel_guide \
  --packages-select dummy-ros2_description dummy_moveit_config dummy_bringup \
  --symlink-install --cmake-args -DBUILD_TESTING=OFF "$@"
# 成功后仅打印新终端的source/launch示例；示例明确选择simulator，但脚本自身不执行该launch。
printf '\n构建完成。新终端执行：\nsource /opt/ros/jazzy/setup.zsh\nsource %s/install_panel_guide/local_setup.zsh\nros2 launch dummy_bringup manual_joint_panel.launch.py backend:=simulator\n' "$PANEL_WORKSPACE"
