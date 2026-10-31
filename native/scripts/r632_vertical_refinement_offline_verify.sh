#!/usr/bin/env bash
# Offline checks only; never starts a ROS node or connects to flight hardware.
set -euo pipefail
ulimit -c 0
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
test_dir="$(mktemp -d)"
trap 'rm -rf -- "$test_dir"' EXIT
for name in test_r632_ramp_schedule test_vertical_takeoff_ceiling \
            test_r632_ramp_height_guard test_r632_height_tracking test_vertical_feedback_regression; do
  g++ -std=c++17 -Wall -Wextra -Werror -pedantic \
    -I "$root/src/jikl_fc01_system/include" "$root/tests/$name.cpp" -o "$test_dir/$name"
  "$test_dir/$name"
done
python3 "$root/tests/adapter_compile_check.py"
echo 'R632_VERTICAL_REFINEMENT_FOCUSED_OFFLINE_PASS (not flight acceptance)'
