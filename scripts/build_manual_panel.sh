#!/usr/bin/env bash
# 明确SDK + 系统Jazzy；只构建面板及ROS控制所需包，不连接设备。
set -eo pipefail
PANEL_WORKSPACE=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
cd "$PANEL_WORKSPACE"
: "${DPSDK_HOST_SDK_PREFIX:?请明确指定既有ROS PIC SDK安装前缀}"
PANEL_SDK_PREFIX=$(realpath "$DPSDK_HOST_SDK_PREFIX")
PANEL_DP_PROTOCOL_DIR="$PANEL_SDK_PREFIX/lib/dp_protocol/lib/cmake/DpProtocol"
PANEL_HOST_SDK_DIR="$PANEL_SDK_PREFIX/lib/cmake/DPSDKHostSDK"
[[ -f "$PANEL_HOST_SDK_DIR/DPSDKHostSDKConfig.cmake" && -f "$PANEL_DP_PROTOCOL_DIR/DpProtocolConfig.cmake" ]] || { echo "所选SDK缺少正式公共CMake包" >&2; exit 2; }
# 公共包身份由上面唯一SDK前缀提供，调用方不能在尾部覆盖成另一个缓存/发布。
for PANEL_EXTRA_ARG in "$@"; do
  case "$PANEL_EXTRA_ARG" in
    -DDPSDK_HOST_SDK_PREFIX*|-DDPSDKHostSDK_DIR*|-DDpProtocol_DIR*|-UDPSDK_HOST_SDK_PREFIX*|-UDPSDKHostSDK_DIR*|-UDpProtocol_DIR*|DPSDK_HOST_SDK_PREFIX=*|DPSDKHostSDK_DIR=*|DpProtocol_DIR=*) echo "禁止额外覆盖所选SDK公共包: $PANEL_EXTRA_ARG" >&2; exit 2;;
  esac
done
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
  --packages-select dummy-ros2_description dummy_moveit_config dummy_motion_interfaces dummy_ros_control dummy_motion_client dummy_bringup \
  --symlink-install --cmake-args -DBUILD_TESTING=OFF -DPython3_EXECUTABLE=/usr/bin/python3 \
  -DDPSDK_HOST_SDK_PREFIX="$PANEL_SDK_PREFIX" \
  -DDPSDKHostSDK_DIR="$PANEL_HOST_SDK_DIR" \
  -DDpProtocol_DIR="$PANEL_DP_PROTOCOL_DIR" "$@"
# 配置后核actual cache；目录存在不足以证明find_package没选择旧overlay。
/usr/bin/python3 - "$PANEL_WORKSPACE/build_panel_guide" "$PANEL_SDK_PREFIX" <<'PYVERIFY'
from pathlib import Path
import sys
base, sdk = map(lambda p: Path(p).resolve(strict=True), sys.argv[1:])
for package in ('dummy_ros_control', 'dummy_bringup'):
    cache = base/package/'CMakeCache.txt'
    fields = dict(line.split('=',1) for line in cache.read_text().splitlines()
                  if '=' in line and not line.startswith(('#','//')))
    for key, suffix in [('DPSDKHostSDK_DIR','lib/cmake/DPSDKHostSDK'),
                        ('DpProtocol_DIR','lib/dp_protocol/lib/cmake/DpProtocol')]:
        matches = [v for k,v in fields.items() if k.split(':',1)[0] == key]
        if len(matches) != 1 or Path(matches[0]).resolve(strict=True) != sdk/suffix:
            raise SystemExit(package + ': 公共包混前缀 ' + key)
        print(package + ': ' + key + '=' + matches[0])
PYVERIFY
printf '\n构建完成：\nsource /opt/ros/jazzy/setup.zsh\nsource %s/install_protocol/local_setup.zsh\nsource %s/install_panel_guide/local_setup.zsh\n' "$PANEL_WORKSPACE" "$PANEL_WORKSPACE"
