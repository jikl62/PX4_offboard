#!/usr/bin/env bash
# Review3 历史拆桨候选的专用构建入口：仅生成隔离 build/install，不启动节点或动作。
# Build only. No ROS node, DDS connection, Arm or Disarm is started here.
set -eo pipefail
review3_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$review3_root"
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo 'ERROR: ROS 2 Humble setup not found at /opt/ros/humble/setup.bash' >&2
  exit 1
fi
source /opt/ros/humble/setup.bash
command -v colcon >/dev/null || { echo 'ERROR: colcon not found' >&2; exit 1; }
# 显式指定真实消息包与当前系统包；该通过消息不是实机验收。
colcon --log-base log_review3 build \
  --base-paths workspace/src/px4_msgs src/jikl_fc01_system \
  --packages-up-to jikl_fc01_system \
  --build-base build_review3 --install-base install_review3 \
  --event-handlers console_direct+
echo 'REVIEW3_NATIVE_BUILD_PASS'
