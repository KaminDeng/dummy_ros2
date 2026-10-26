#!/usr/bin/env bash
# 明确SDK + 系统Jazzy；只构建面板及ROS控制所需包，不连接设备。
set -eo pipefail
PANEL_WORKSPACE=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
cd "$PANEL_WORKSPACE"
: "${DPSDK_HOST_SDK_PREFIX:?请明确指定既有ROS PIC SDK安装前缀}"
PANEL_SDK_PREFIX=$(realpath "$DPSDK_HOST_SDK_PREFIX")
PANEL_PROTOCOL_TOOLS=${DPSDK_PROTOCOL_TOOLS:-${DPSDK_ROOT:-$PANEL_WORKSPACE/../dpsdk}/libraries/dp_protocol/tools}
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH
source /opt/ros/jazzy/setup.bash
set -u
/usr/bin/python3 scripts/assemble_sdk_ros_interfaces.py \
  --sdk-prefix "$PANEL_SDK_PREFIX" --protocol-tools "$PANEL_PROTOCOL_TOOLS" \
  --output build_protocol/interfaces/dpsdk_interfaces
colcon --log-base log_protocol build --base-paths build_protocol/interfaces \
  --build-base build_protocol --install-base install_protocol --packages-select dpsdk_interfaces \
  --cmake-args -DBUILD_TESTING=OFF
source install_protocol/local_setup.bash
colcon --log-base log_panel_guide build \
  --base-paths src --build-base build_panel_guide --install-base install_panel_guide \
  --packages-select dummy-ros2_description dummy_moveit_config dummy_ros_control dummy_motion_client dummy_bringup \
  --symlink-install --cmake-args -DBUILD_TESTING=OFF -DPython3_EXECUTABLE=/usr/bin/python3 \
  -DDPSDK_HOST_SDK_PREFIX="$PANEL_SDK_PREFIX" \
  -DDPSDKHostSDK_DIR="$PANEL_SDK_PREFIX/lib/cmake/DPSDKHostSDK" "$@"
printf '\n构建完成：\nsource /opt/ros/jazzy/setup.zsh\nsource %s/install_protocol/local_setup.zsh\nsource %s/install_panel_guide/local_setup.zsh\n' "$PANEL_WORKSPACE" "$PANEL_WORKSPACE"
