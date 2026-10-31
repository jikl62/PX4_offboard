#!/usr/bin/env bash
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$root/scripts/common.sh"
profile="${1:-${FC01_PROFILE:-ground}}"
[[ "$profile" == ground || "$profile" == flight || "$profile" == sim || "$profile" == attitude_altitude ]] || { echo 'profile: ground/flight/sim/attitude_altitude' >&2; exit 2; }
[[ "$profile" != sim || "$ROS_DOMAIN_ID" != 0 ]] || { echo 'sim 必须使用独立 ROS 域，并让 SITL 的 UXRCE_DDS_DOM_ID 与之匹配；禁止默认域0混接。' >&2; exit 2; }
workspace="$root/workspace"
[[ -f "$workspace/install/local_setup.bash" ]] || { echo '请先运行 install.sh，然后从新版本/current 启动。' >&2; exit 2; }
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH ROS_PACKAGE_PATH ROS_DISTRO
set +u
source "${ROS_SETUP:-/opt/ros/humble/setup.bash}"
source "$workspace/install/local_setup.bash"
set -u
log_directory="${FC01_LOG_DIR:-$FC01_BASE/logs}"
mkdir -p "$log_directory"
csv="$log_directory/telemetry_$(date -u +%Y%m%dT%H%M%SZ).csv"
exec "$workspace/install/jikl_fc01_system/lib/jikl_fc01_system/fc01_system_node" --ros-args --params-file "$root/config/$profile.yaml" -p "csv_path:=$csv"
