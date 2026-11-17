#!/usr/bin/env bash
# 待机自启命令的薄入口：根据自身目录定位 Python 实现，exec 保持信号与退出码直达。
set -e
fc01_boot_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
# 所有参数原样传给 autostart.py；服务安装/回滚权限在 Python 层检查。
exec /usr/bin/python3 "$fc01_boot_dir/autostart.py" "$@"
