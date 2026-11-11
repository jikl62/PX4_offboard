#!/usr/bin/env bash
# 历史待机服务入口：加载 Humble 和指定 SDK，运行 standby.py 管理原生进程及状态窗口。
set -e
fc01_boot_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
fc01_native_dir=${FC01_PROJECT_ROOT:-"$HOME/FC01_ARM64_PORT_20261020/FC01_R632_P0_ATTITUDE_AUDIT_CANDIDATE"}
# 这里依赖用户已安装环境；找不到时 set -e 中止，不偷偷安装依赖或改串口。
source /opt/ros/humble/setup.bash
source "$fc01_native_dir/install_r632/local_setup.bash"
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 PYTHONUNBUFFERED=1
# 转交信号给待机监督器，由其负责节点先退出、Agent 后退出的生命周期。
exec /usr/bin/python3 "$fc01_boot_dir/standby.py" "$@"
