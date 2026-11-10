#!/usr/bin/env bash
# Strict handling stays inside this script, never in the operator's shell.
set -euo pipefail
fc01_deploy_entry=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
exec /usr/bin/python3 "$fc01_deploy_entry/navigation_deploy.py" "$@"
