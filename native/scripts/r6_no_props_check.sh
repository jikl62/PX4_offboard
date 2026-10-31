#!/usr/bin/env bash
# Local check of files captured during a no-props check. Does NOT connect to
# FC01/DDS, start a controller, open UART, Arm/Disarm, or change PX4 parameters.
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
python3 "$root/tools/r6_preflight_audit.py" \
  --config "$root/config/R6_REAL_FC01_PENDING_REVIEW.yaml" "$@"
