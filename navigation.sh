#!/usr/bin/env bash
# 正式导航 CLI 的薄入口：只定位同目录 navigation.py，具体命令/权限由 Python 层验证。
set -e
fc01_navigation_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
# 用 exec 替换 shell，保留 systemd/终端对真实进程的信号和退出码。
exec /usr/bin/python3 "$fc01_navigation_dir/navigation.py" "$@"
