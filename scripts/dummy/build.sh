#!/usr/bin/env bash
# @file scripts/dummy/build.sh
# @brief 校验相邻DPSDK来源、生成场景SDF并构建ROS工作区。
# @details 与三包面板脚本不同，此入口未指定packages-select，colcon按工作区发现结果构建。
# 它会改写受控empty.sdf并生成构建产物，不启动控制节点或申请运动许可。
set -euo pipefail

# 由脚本自身位置找工作区；DPSDK_ROOT优先使用调用方环境，否则尝试相邻dpsdk。
# cd失败的候选查询允许回退，随后明确检查zenbedded.repos存在；这只是路径锚，不证明所有源/库版本一致。
workspace_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
dpsdk_candidate=$(cd "$workspace_root/../dpsdk" 2>/dev/null && pwd || true)
DPSDK_ROOT="${DPSDK_ROOT:-$dpsdk_candidate}"
if [[ -z "$DPSDK_ROOT" || ! -f "$DPSDK_ROOT/tools/ros2/zenbedded.repos" ]]; then
  echo "invalid DPSDK_ROOT: $DPSDK_ROOT" >&2
  exit 2
fi
export DPSDK_ROOT
cd "$workspace_root"
# 可显式隔离本轮SDK消费产物，默认目录保持既有入口兼容。
protocol_build_base="${DUMMY_PROTOCOL_BUILD_BASE:-build_protocol}"
protocol_install_base="${DUMMY_PROTOCOL_INSTALL_BASE:-install_protocol}"
protocol_log_base="${DUMMY_PROTOCOL_LOG_BASE:-log_protocol}"
dummy_build_base="${DUMMY_BUILD_BASE:-build_dummy}"
dummy_install_base="${DUMMY_INSTALL_BASE:-install_dummy}"
dummy_log_base="${DUMMY_LOG_BASE:-log_dummy}"
: "${DPSDK_HOST_SDK_PREFIX:?请明确指定既有ROS PIC SDK安装前缀}"
/usr/bin/python3 scripts/assemble_sdk_ros_interfaces.py \
  --sdk-prefix "$DPSDK_HOST_SDK_PREFIX" \
  --protocol-tools "${DPSDK_PROTOCOL_TOOLS:-$DPSDK_ROOT/libraries/dp_protocol/tools}" \
  --output "$protocol_build_base/interfaces/dpsdk_interfaces"
colcon --log-base "$protocol_log_base" build --base-paths "$protocol_build_base/interfaces" \
  --build-base "$protocol_build_base" --install-base "$protocol_install_base" --packages-select dpsdk_interfaces
# colcon生成的setup读取可选未设置变量，临时关闭nounset后恢复本脚本约束。
set +u
source "$protocol_install_base/local_setup.bash"
set -u
# 从canonical scene.yaml重新生成empty.sdf；应修改场景输入或模板来维护生成内容。
/usr/bin/python3 src/dummy_bringup/scripts/generate_world.py \
  --scene src/dummy_bringup/config/scene.yaml \
  --output src/dummy_moveit_config/worlds/empty.sdf
# 固定系统Python以匹配ROS系统包，使用build/install/log_dummy；其余构建参数原样转发。
# 该脚本不主动清除历史underlay，调用前需要准备正确ROS和SDK依赖环境。
colcon --log-base "$dummy_log_base" build --build-base "$dummy_build_base" --install-base "$dummy_install_base" \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 \
  -DDPSDK_HOST_SDK_PREFIX="$DPSDK_HOST_SDK_PREFIX" \
  -DDPSDKHostSDK_DIR="$DPSDK_HOST_SDK_PREFIX/lib/cmake/DPSDKHostSDK" "$@"
