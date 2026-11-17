#!/usr/bin/env bash
# 旧交互起飞入口：尝试加载环境后进入 launcher 的自动检查/报告流程，不直接发布任何飞控指令。
set -e
fc01_entry_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
fc01_project_dir=${FC01_PROJECT_ROOT:-"$HOME/FC01_ARM64_PORT_20261020/FC01_R632_P0_ATTITUDE_AUDIT_CANDIDATE"}
# 环境缺失仍进入 Python 报告，而不是在 shell 层绕过检查或自动解锁。
if [[ ! -f /opt/ros/humble/setup.bash || ! -f "$fc01_project_dir/install_r632/local_setup.bash" ]]; then
  printf '%s\n' '未找到已安装的 Humble / install_r632 环境；进入自动报告。' >&2
else
  source /opt/ros/humble/setup.bash || printf '%s\n' '加载 Humble 失败，进入自动检查和报告。' >&2
  source "$fc01_project_dir/install_r632/local_setup.bash" || printf '%s\n' '加载 install_r632 失败，进入自动检查和报告。' >&2
fi
# 保留用户显式设置的 DDS 域/网络范围；默认值沿用当前链路。
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0} ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-0}
exec /usr/bin/python3 "$fc01_entry_dir/launcher.py" "$@"
