#!/usr/bin/env bash
# Native offline build/reuse only: no controller, Agent, probe or systemd action.
set -Eeuo pipefail
fc01_build_entry=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
mkdir -p "$HOME/FC01_navigation"
fc01_build_log=$(mktemp "$HOME/FC01_navigation/offline-build-$(date -u +%Y%m%dT%H%M%SZ)-XXXXXX.log")
export FC01_BUILD_LOG="$fc01_build_log"
exec > >(tee -a "$fc01_build_log") 2>&1
trap 'fc01_build_status=$?; echo "FC01_OFFLINE_BUILD_FAILED stage=${fc01_build_stage:-unknown} exit=$fc01_build_status log=$fc01_build_log" >&2; exit "$fc01_build_status"' ERR
printf 'FC01_OFFLINE_BUILD_LOG=%s\n' "$fc01_build_log"
fc01_build_stage=environment
source "$fc01_build_entry/navigation_environment.sh"
fc01_load_environment "$fc01_build_entry"
fc01_build_stage=build
/usr/bin/python3 "$fc01_build_entry/navigation_build_evidence.py" build --entry "$fc01_build_entry"
