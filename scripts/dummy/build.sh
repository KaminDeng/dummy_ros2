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
# 从canonical scene.yaml重新生成empty.sdf；应修改场景输入或模板来维护生成内容。
/usr/bin/python3 src/dummy_bringup/scripts/generate_world.py \
  --scene src/dummy_bringup/config/scene.yaml \
  --output src/dummy_moveit_config/worlds/empty.sdf
# 固定系统Python以匹配ROS系统包，使用build/install/log_dummy；其余构建参数原样转发。
# 该脚本不主动清除历史underlay，调用前需要准备正确ROS和SDK依赖环境。
colcon --log-base log_dummy build --build-base build_dummy --install-base install_dummy \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 "$@"
