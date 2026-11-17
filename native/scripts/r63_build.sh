#!/usr/bin/env bash
# R6.3 历史候选 ROS 构建：使用独立目录，不运行 DDS、控制节点或电机。
# R6.3 offline/native build. Does not start nodes, DDS, or motors.
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$root"
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "ROS2_HUMBLE_NOT_INSTALLED: run this native build on the FC01 Ubuntu host" >&2
  exit 1
fi
set +u  # ROS Humble setup.bash assumes unset AMENT_TRACE_SETUP_FILES is permitted
source /opt/ros/humble/setup.bash
set -u
# 构建到目标包及其消息依赖，命令成功只说明原生编译完成。
colcon --log-base log_r63 build \
  --base-paths workspace/src/px4_msgs src/jikl_fc01_system \
  --packages-up-to jikl_fc01_system \
  --build-base build_r63 --install-base install_r63 \
  --event-handlers console_direct+
echo R63_NATIVE_BUILD_PASS
