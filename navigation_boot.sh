#!/usr/bin/env bash
set -e
fc01_navigation_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
fc01_base_dir=${FC01_PROJECT_ROOT:-"$HOME/FC01_ARM64_PORT_20261020/FC01_R632_P0_ATTITUDE_AUDIT_CANDIDATE"}
source /opt/ros/humble/setup.bash
source "$fc01_base_dir/install_r632/local_setup.bash"
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 PYTHONUNBUFFERED=1
exec /usr/bin/python3 "$fc01_navigation_dir/navigation_service.py" "$@"
