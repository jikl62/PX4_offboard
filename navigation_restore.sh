#!/usr/bin/env bash
# 旧部署恢复入口：使用同一 SDK 解析器，再把解析结果显式传给恢复工具。
set -euo pipefail
fc01_restore_entry=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
source "$fc01_restore_entry/navigation_environment.sh"
fc01_load_environment "$fc01_restore_entry"
# 恢复的地面证据、托管服务身份和人工确认仍由 navigation_restore.py 校验。
exec /usr/bin/python3 "$fc01_restore_entry/navigation_restore.py" "$@" --project-root "$FC01_PROJECT_ROOT"
