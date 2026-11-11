#!/usr/bin/env bash
# 原生生命周期监督器的薄入口：共享环境后 exec start.py，参数与信号不经二次解释。
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$root/scripts/common.sh"
exec python3 "$root/tools/start.py" "$@"
