#!/usr/bin/env bash
# R6.3 focused regression. Never connects to the aircraft.
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
bin="$(mktemp)"; trap 'rm -f -- "$bin"' EXIT
g++ -std=c++17 -Wall -Wextra -Werror -pedantic \
  -I "$root/src/jikl_fc01_system/include" \
  "$root/tests/test_r63_takeoff_candidate.cpp" -o "$bin"
"$bin"
g++ -std=c++17 -Wall -Wextra -Werror -pedantic \
  -I "$root/src/jikl_fc01_system/include" \
  "$root/tests/test_r63_normal_arm_evidence.cpp" -o "$bin"
"$bin"
python3 "$root/tests/adapter_compile_check.py"
python3 "$root/tools/r63_config_check.py" "$root/config/attitude_altitude.yaml"
echo R63_FOCUSED_OFFLINE_PASS
