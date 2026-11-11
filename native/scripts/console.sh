#!/usr/bin/env bash
# 板载控制台入口：复用共同环境，只运行 fcctl.py；具体 Arm/Land/接管检查仍在协议与 Runtime。
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$root/scripts/common.sh"
exec python3 "$root/tools/fcctl.py" "$@"
