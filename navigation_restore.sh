#!/usr/bin/env bash
set -euo pipefail
fc01_restore_entry=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
source "$fc01_restore_entry/navigation_environment.sh"
fc01_load_environment "$fc01_restore_entry"
exec /usr/bin/python3 "$fc01_restore_entry/navigation_restore.py" "$@" --project-root "$FC01_PROJECT_ROOT"
