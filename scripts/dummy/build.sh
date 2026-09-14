#!/usr/bin/env bash
set -euo pipefail

workspace_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
cd "$workspace_root"
/usr/bin/python3 src/dummy_bringup/scripts/generate_world.py \
  --scene src/dummy_bringup/config/scene.yaml \
  --output src/dummy_moveit_config/worlds/empty.sdf
DPSDK_ROOT=../dpsdk \
  colcon --log-base log_dummy build --build-base build_dummy --install-base install_dummy \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 "$@"
