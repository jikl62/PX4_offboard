#!/usr/bin/env bash
# Source only the pinned bundle overlay, then replay inherited DDS/RMW settings.
set -eo pipefail
export PYTHONDONTWRITEBYTECODE=1
fc01_prebuilt_entry=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
fc01_prebuilt_env=$(mktemp)
trap 'rm -f -- "$fc01_prebuilt_env"' EXIT
/usr/bin/python3 "$fc01_prebuilt_entry/navigation_prebuilt_service.py" environment \
  --entry "$fc01_prebuilt_entry" > "$fc01_prebuilt_env"
mapfile -d '' -t fc01_prebuilt_values < "$fc01_prebuilt_env"
rm -f -- "$fc01_prebuilt_env"
trap - EXIT
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH \
  ROS_PACKAGE_PATH ROS_DISTRO COLCON_CURRENT_PREFIX AMENT_CURRENT_PREFIX
[[ -f /opt/ros/humble/setup.bash && -f "$fc01_prebuilt_entry/install/local_setup.bash" ]] || {
  echo 'FC01_PREBUILT_SERVICE_FAILED: ROS Humble or bundled overlay is missing' >&2
  exit 2
}
source /opt/ros/humble/setup.bash
source "$fc01_prebuilt_entry/install/local_setup.bash"
for fc01_prebuilt_pair in "${fc01_prebuilt_values[@]}"; do
  export "$fc01_prebuilt_pair"
done
export PYTHONUNBUFFERED=1 PYTHONDONTWRITEBYTECODE=1
exec /usr/bin/python3 "$fc01_prebuilt_entry/navigation_prebuilt_service.py" "$@"
