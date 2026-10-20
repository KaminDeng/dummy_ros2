#!/usr/bin/env bash
# 只构建手动面板所需的三个本地包；安装区可独立用于 real / simulator。
set -eo pipefail
PANEL_WORKSPACE=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
cd "$PANEL_WORKSPACE"
# 避免误用历史 ROS 工作区作为 underlay；只依赖系统 Jazzy。
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH
source /opt/ros/jazzy/setup.bash
set -u
colcon --log-base log_panel_guide build \
  --base-paths src/dummy-ros2_description src/dummy_moveit_config src/dummy_bringup \
  --build-base build_panel_guide --install-base install_panel_guide \
  --packages-select dummy-ros2_description dummy_moveit_config dummy_bringup \
  --symlink-install --cmake-args -DBUILD_TESTING=OFF "$@"
printf '\n构建完成。新终端执行：\nsource /opt/ros/jazzy/setup.zsh\nsource %s/install_panel_guide/local_setup.zsh\nros2 launch dummy_bringup manual_joint_panel.launch.py backend:=simulator\n' "$PANEL_WORKSPACE"
