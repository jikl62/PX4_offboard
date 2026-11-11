#!/usr/bin/env bash
# 原生节点启动入口：验证 profile、隔离仿真域、加载此版本 workspace，并给该进程指定 CSV 日志。
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$root/scripts/common.sh"
profile="${1:-${FC01_PROFILE:-ground}}"
# profile 只允许显式白名单；仿真必须独立域，防止 SITL 与实机域 0 混接。
[[ "$profile" == ground || "$profile" == flight || "$profile" == sim || "$profile" == attitude_altitude ]] || { echo 'profile: ground/flight/sim/attitude_altitude' >&2; exit 2; }
[[ "$profile" != sim || "$ROS_DOMAIN_ID" != 0 ]] || { echo 'sim 必须使用独立 ROS 域，并让 SITL 的 UXRCE_DDS_DOM_ID 与之匹配；禁止默认域0混接。' >&2; exit 2; }
workspace="$root/workspace"
[[ -f "$workspace/install/local_setup.bash" ]] || { echo '请先运行 install.sh，然后从新版本/current 启动。' >&2; exit 2; }
# 移除继承 overlay 路径后按当前版本重新加载，避免误用旧消息/二进制。
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH ROS_PACKAGE_PATH ROS_DISTRO
set +u
source "${ROS_SETUP:-/opt/ros/humble/setup.bash}"
source "$workspace/install/local_setup.bash"
set -u
log_directory="${FC01_LOG_DIR:-$FC01_BASE/logs}"
mkdir -p "$log_directory"
# 该历史入口启用同步 CSV；正式导航服务的 csv_path 策略在另一入口，不能混为一谈。
csv="$log_directory/telemetry_$(date -u +%Y%m%dT%H%M%SZ).csv"
exec "$workspace/install/jikl_fc01_system/lib/jikl_fc01_system/fc01_system_node" --ros-args --params-file "$root/config/$profile.yaml" -p "csv_path:=$csv"
