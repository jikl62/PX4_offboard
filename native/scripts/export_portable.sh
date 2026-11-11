#!/usr/bin/env bash
# 便携源码包入口：把所选真实 px4_msgs 与原生源码导出，不编译、不激活服务。
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
# 消息来源路径由显式参数优先决定；导出器会检查 schema/内容，不能凭同名目录认定兼容。
messages="${1:-$HOME/fc01_real_ws/src/px4_msgs}"
output="${2:-$HOME/fc01_portable_$(cat "$root/VERSION").zip}"
exec python3 "$root/tools/deploy.py" export "$messages" "$output"
