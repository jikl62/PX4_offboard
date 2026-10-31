#!/usr/bin/env bash
# Native ROS2 build only. This never starts FC01 nodes or commands.
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$root"
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "ROS2_HUMBLE_NOT_INSTALLED: build on FC01 Ubuntu host" >&2
  exit 1
fi
set +u
source /opt/ros/humble/setup.bash
set -u
colcon --log-base log_r632 build \
  --base-paths workspace/src/px4_msgs src/jikl_fc01_system \
  --packages-up-to jikl_fc01_system \
  --build-base build_r632 --install-base install_r632 \
  --event-handlers console_direct+
echo R632_NATIVE_BUILD_PASS
