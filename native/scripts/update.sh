#!/usr/bin/env bash
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$root/scripts/common.sh"
release="${1:-$(cat "$FC01_BASE/built-release")}";release="$(realpath -- "$release")"
[[ "$release" == "$FC01_BASE/releases/"* && -x "$release/workspace/install/jikl_fc01_system/lib/jikl_fc01_system/fc01_system_node" ]] || { echo '新版本未构建或路径不在 releases 下。' >&2; exit 2; }
# Diagnostic-only for every existing socket, including stale-looking ones.
# Do not connect to a live console merely to diagnose it: disconnect has control
# semantics. Quiescing is explicit and outside this activation script.
python3 "$root/tools/update.py" "$FC01_BASE" "$release" "$FC01_SOCKET"
echo "已切换到 $release；新进程从空闲开始，不恢复旧任务。"
