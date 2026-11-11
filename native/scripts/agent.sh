#!/usr/bin/env bash
# DDS 串口 Agent 启动器：核对设备访问与占用，选择受控原生 Agent，再替换当前 shell。
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$root/scripts/common.sh"
# 设备必须是可读写字符设备；占用诊断只提示，不停止其他所有者的进程。
[[ -c "$FC01_DEVICE" && -r "$FC01_DEVICE" && -w "$FC01_DEVICE" ]] || { echo "串口不存在或不可读写：$FC01_DEVICE；核对线缆和 dialout 权限。" >&2; exit 2; }
if fuser "$FC01_DEVICE" >/dev/null 2>&1; then fuser -v "$FC01_DEVICE" >&2; echo '串口已占用。已有 Agent 时用 start.sh --external-agent；其他程序请核实后停止。' >&2; exit 2; fi
# 优先显式 FC01_AGENT，其次本工程依赖和既有工作区，最后才查 PATH。
agent="${FC01_AGENT:-}"
if [[ -z "$agent" ]]; then
  for candidate in "$FC01_BASE/deps/bin/MicroXRCEAgent" "$HOME/ros2_px4_ws/install/microxrcedds_agent/bin/MicroXRCEAgent"; do
    if [[ -x "$candidate" ]]; then agent="$candidate"; break; fi
  done
fi
if [[ -z "$agent" ]]; then agent="$(command -v MicroXRCEAgent || true)"; fi
[[ -n "$agent" && -x "$agent" ]] || { echo '未找到原生 Agent；先运行 build_agent.sh。' >&2; exit 2; }
# 把此版本依赖库加入本 Agent 环境，不修改系统库或父进程环境。
export LD_LIBRARY_PATH="$FC01_BASE/deps/lib:$FC01_BASE/deps/lib64${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
exec "$agent" serial --dev "$FC01_DEVICE" -b "$FC01_BAUD"
