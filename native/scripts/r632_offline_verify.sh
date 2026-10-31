#!/usr/bin/env bash
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
bin="$(mktemp)"
trap 'rm -f -- "$bin"' EXIT
g++ -std=c++17 -Wall -Wextra -Werror -pedantic \
  -I "$root/src/jikl_fc01_system/include" \
  "$root/tests/test_r632_bounded_liftoff_reserve.cpp" -o "$bin"
"$bin"
bash "$root/scripts/r63_offline_verify.sh"
bash "$root/scripts/r632_ramp_guard_offline_verify.sh"
