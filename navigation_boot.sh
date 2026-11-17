#!/usr/bin/env bash
# 导航服务的环境入口：先解析已安装 SDK，再执行服务进程；不自行生成动作授权。
set -e
fc01_navigation_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
# 加载函数只影响当前部署子进程；不能污染操作者的父 shell。
source "$fc01_navigation_dir/navigation_environment.sh"
fc01_load_environment "$fc01_navigation_dir"
# run/prepare-config/build 等命令仍由 navigation_service 的显式子命令分派。
exec /usr/bin/python3 "$fc01_navigation_dir/navigation_service.py" "$@"
