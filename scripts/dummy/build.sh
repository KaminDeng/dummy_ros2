#!/usr/bin/env bash
set -euo pipefail

workspace_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
cd "$workspace_root"
DPSDK_ROOT=../dpsdk \
  colcon --log-base log_dummy build --build-base build_dummy --install-base install_dummy \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 "$@"
