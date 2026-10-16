#!/usr/bin/env bash
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$root/scripts/common.sh"
[[ -c "$FC01_DEVICE" && -r "$FC01_DEVICE" && -w "$FC01_DEVICE" ]] || { echo "串口不存在或不可读写：$FC01_DEVICE；核对线缆和 dialout 权限。" >&2; exit 2; }
if fuser "$FC01_DEVICE" >/dev/null 2>&1; then fuser -v "$FC01_DEVICE" >&2; echo '串口已占用。已有 Agent 时用 start.sh --external-agent；其他程序请核实后停止。' >&2; exit 2; fi
agent="${FC01_AGENT:-}"
if [[ -z "$agent" ]]; then
  for candidate in "$FC01_BASE/deps/bin/MicroXRCEAgent" "$HOME/ros2_px4_ws/install/microxrcedds_agent/bin/MicroXRCEAgent"; do
    if [[ -x "$candidate" ]]; then agent="$candidate"; break; fi
  done
fi
if [[ -z "$agent" ]]; then agent="$(command -v MicroXRCEAgent || true)"; fi
[[ -n "$agent" && -x "$agent" ]] || { echo '未找到原生 Agent；先运行 build_agent.sh。' >&2; exit 2; }
export LD_LIBRARY_PATH="$FC01_BASE/deps/lib:$FC01_BASE/deps/lib64${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
exec "$agent" serial --dev "$FC01_DEVICE" -b "$FC01_BAUD"
