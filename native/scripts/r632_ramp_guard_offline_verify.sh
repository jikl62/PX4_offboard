#!/usr/bin/env bash
set -euo pipefail
ulimit -c 0
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
test_dir="$(mktemp -d)"
trap 'rm -rf -- "$test_dir"' EXIT
for name in test_r632_ramp_height_guard test_r632_height_tracking test_r632_takeoff_plant; do
  g++ -std=c++17 -Wall -Wextra -Werror -pedantic \
    -I "$root/src/jikl_fc01_system/include" "$root/tests/$name.cpp" -o "$test_dir/$name"
  if [[ "$name" == test_r632_takeoff_plant ]]; then
    "$test_dir/$name" > "$test_dir/nominal_plant.csv"
  else
    "$test_dir/$name"
  fi
done
# Optional, separate sensitivity report includes known low-hover failures.
# It never counts those cases as flight-acceptance passes.
if [[ "${1:-}" == --sensitivity ]]; then
  if [[ -z "${2:-}" ]]; then
    echo 'usage: bash scripts/r632_ramp_guard_offline_verify.sh --sensitivity OUTPUT.csv' >&2
    exit 2
  fi
  "$test_dir/test_r632_takeoff_plant" sensitivity > "$2"
fi
echo 'R632_RAMP_GUARD_FOCUSED_OFFLINE_PASS (no ROS/PX4/hardware flight claim)'
