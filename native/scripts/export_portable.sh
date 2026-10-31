#!/usr/bin/env bash
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
messages="${1:-$HOME/fc01_real_ws/src/px4_msgs}"
output="${2:-$HOME/fc01_portable_$(cat "$root/VERSION").zip}"
exec python3 "$root/tools/deploy.py" export "$messages" "$output"
