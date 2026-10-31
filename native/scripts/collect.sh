#!/usr/bin/env bash
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$root/scripts/common.sh"
out="$FC01_BASE/evidence_$(date -u +%Y%m%dT%H%M%SZ)"
mkdir -p "$out"
cp /etc/os-release "$out/os-release"
uname -m > "$out/architecture.txt"
systemctl --user status fc01-system.service --no-pager > "$out/service.txt" 2>&1 || true
fuser -v "$FC01_DEVICE" > "$out/serial-owner.txt" 2>&1 || true
python3 "$root/tools/fcctl.py" --socket "$FC01_SOCKET" --command status --json > "$out/status.jsonl" 2> "$out/status-error.txt" || true
if [[ -d "$FC01_BASE/logs" ]]; then cp -a "$FC01_BASE/logs" "$out/logs"; fi
tar -czf "$out.tar.gz" -C "$FC01_BASE" "$(basename "$out")"
echo "采集完成：$out.tar.gz"
