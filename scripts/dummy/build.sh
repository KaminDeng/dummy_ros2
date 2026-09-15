#!/usr/bin/env bash
set -euo pipefail

workspace_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
dpsdk_candidate=$(cd "$workspace_root/../dpsdk" 2>/dev/null && pwd || true)
DPSDK_ROOT="${DPSDK_ROOT:-$dpsdk_candidate}"
if [[ -z "$DPSDK_ROOT" || ! -f "$DPSDK_ROOT/tools/ros2/zenbedded.repos" ]]; then
  echo "invalid DPSDK_ROOT: $DPSDK_ROOT" >&2
  exit 2
fi
export DPSDK_ROOT
cd "$workspace_root"
/usr/bin/python3 src/dummy_bringup/scripts/generate_world.py \
  --scene src/dummy_bringup/config/scene.yaml \
  --output src/dummy_moveit_config/worlds/empty.sdf
colcon --log-base log_dummy build --build-base build_dummy --install-base install_dummy \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 "$@"
