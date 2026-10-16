#!/usr/bin/env bash
# Offline C++ unit tests and API syntax fixture; does not access PX4 or hardware.
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
workspace="$(mktemp -d)"
trap 'rm -rf -- "$workspace"' EXIT
pass=0
for test_file in "$root"/tests/test_*.cpp; do
  base="$(basename "$test_file" .cpp)"
  g++ -std=c++17 -Wall -Wextra -Werror -pedantic \
    -I "$root/src/jikl_fc01_system/include" "$test_file" -o "$workspace/$base"
  "$workspace/$base"
  pass=$((pass+1))
done
python3 "$root/tests/adapter_compile_check.py"
python3 "$root/scripts/r62_failure_replay.py"
echo "R62_CPP_TESTS_PASS=$pass (NOT a native ROS build or flight release)"
