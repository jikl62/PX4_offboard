#!/usr/bin/env bash
# R6.3.2 原生 ROS 构建入口：需要已安装 Humble，产物位于独立 build/install_r632。
# Native ROS2 build only. This never starts FC01 nodes or commands.
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$root"
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "ROS2_HUMBLE_NOT_INSTALLED: build on FC01 Ubuntu host" >&2
  exit 1
fi
# ROS setup 与 nounset 不完全兼容，加载后恢复严格未设置变量检查。
set +u
source /opt/ros/humble/setup.bash
set -u
# 同时使用当前工程包和指定 workspace 内 px4_msgs，不运行新生成的节点。
colcon --log-base log_r632 build \
  --base-paths workspace/src/px4_msgs src/jikl_fc01_system \
  --packages-up-to jikl_fc01_system \
  --build-base build_r632 --install-base install_r632 \
  --event-handlers console_direct+
echo R632_NATIVE_BUILD_PASS
