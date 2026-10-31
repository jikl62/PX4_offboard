#!/usr/bin/env bash
set -e
fc01_entry_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
fc01_project_dir=${FC01_PROJECT_ROOT:-"$HOME/FC01_ARM64_PORT_20261020/FC01_R632_P0_ATTITUDE_AUDIT_CANDIDATE"}
if [[ ! -f /opt/ros/humble/setup.bash || ! -f "$fc01_project_dir/install_r632/local_setup.bash" ]]; then
  printf '%s\n' '未找到已安装的 Humble / install_r632 环境；进入自动报告。' >&2
else
  source /opt/ros/humble/setup.bash || printf '%s\n' '加载 Humble 失败，进入自动检查和报告。' >&2
  source "$fc01_project_dir/install_r632/local_setup.bash" || printf '%s\n' '加载 install_r632 失败，进入自动检查和报告。' >&2
fi
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0} ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-0}
exec /usr/bin/python3 "$fc01_entry_dir/launcher.py" "$@"
