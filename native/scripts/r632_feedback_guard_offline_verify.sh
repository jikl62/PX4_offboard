#!/usr/bin/env bash
# Offline only: no ROS nodes, device access, arming or setpoint publication.
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
test_dir="$(mktemp -d)"
trap 'rm -rf -- "$test_dir"' EXIT
for name in test_vertical_feedback_guard test_vertical_feedback_regression test_r632_takeoff_plant; do
  g++ -std=c++17 -Wall -Wextra -Werror -pedantic \
    -I "$root/src/jikl_fc01_system/include" "$root/tests/$name.cpp" -o "$test_dir/$name"
  if [[ "$name" == test_r632_takeoff_plant ]]; then
    "$test_dir/$name" guard > "$test_dir/guarded_plant.csv"
  else
    "$test_dir/$name"
  fi
done
python3 "$root/tests/adapter_compile_check.py"
echo 'R632_HEIGHT_FEEDBACK_GUARD_FOCUSED_PASS (offline only; no flight acceptance)'
