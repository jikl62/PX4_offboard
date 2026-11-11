#!/usr/bin/env bash
# 板载证据采集：保存系统、服务、串口所有者、控制状态和日志，再打包为独立 tar.gz。
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$root/scripts/common.sh"
out="$FC01_BASE/evidence_$(date -u +%Y%m%dT%H%M%SZ)"
mkdir -p "$out"
cp /etc/os-release "$out/os-release"
uname -m > "$out/architecture.txt"
# 各诊断命令失败仍保留其它材料；status 请求只读，不作为飞控动作授权。
systemctl --user status fc01-system.service --no-pager > "$out/service.txt" 2>&1 || true
fuser -v "$FC01_DEVICE" > "$out/serial-owner.txt" 2>&1 || true
python3 "$root/tools/fcctl.py" --socket "$FC01_SOCKET" --command status --json > "$out/status.jsonl" 2> "$out/status-error.txt" || true
if [[ -d "$FC01_BASE/logs" ]]; then cp -a "$FC01_BASE/logs" "$out/logs"; fi
# 归档本次新建目录，不清理运行日志或控制 socket。
tar -czf "$out.tar.gz" -C "$FC01_BASE" "$(basename "$out")"
echo "采集完成：$out.tar.gz"
