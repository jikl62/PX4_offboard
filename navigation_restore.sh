#!/usr/bin/env bash
set -e
fc01_restore_entry=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
fc01_restore_sdk=${FC01_PROJECT_ROOT:?Set FC01_PROJECT_ROOT to the existing ROS/px4_msgs SDK root}
source /opt/ros/humble/setup.bash
source "$fc01_restore_sdk/install_r632/local_setup.bash"
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 PYTHONUNBUFFERED=1
exec /usr/bin/python3 "$fc01_restore_entry/navigation_restore.py" "$@" --project-root "$fc01_restore_sdk"
