#!/usr/bin/env bash
# 拆桨记录的本地复核入口：只检查已捕获配置和事件文件，不打开 UART/DDS 或修改 PX4。
# Local check of files captured during a no-props check. Does NOT connect to
# FC01/DDS, start a controller, open UART, Arm/Disarm, or change PX4 parameters.
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
python3 "$root/tools/r6_preflight_audit.py" \
  --config "$root/config/R6_REAL_FC01_PENDING_REVIEW.yaml" "$@"
