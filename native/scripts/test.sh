#!/usr/bin/env bash
set -euo pipefail
ulimit -c 0
export PYTHONDONTWRITEBYTECODE=1
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
test_dir="$(mktemp -d)"
trap 'rm -rf -- "$test_dir"' EXIT
inc="$root/src/jikl_fc01_system/include"
for test_file in "$root"/tests/test_*.cpp; do
  bin="$test_dir/$(basename "$test_file" .cpp)"
  g++ -std=c++17 -Wall -Wextra -Werror -pedantic -I "$inc" "$test_file" -o "$bin"
  "$bin"
done
# C's independent raw reproduction sources remain byte-for-byte unchanged.
# Treat their stdout as preserved evidence, and rely on the assertion-driven
# R3/R4 positive safety tests for pass/fail interpretation.
for repro in "$root"/tests/c_independent_repro/C_M1_COUNTEREXAMPLES.cpp \
             "$root"/tests/c_independent_repro/C_R2_NEW_INTERLEAVINGS.cpp \
             "$root"/tests/c_independent_repro/C_R2_SLEW_AND_ACK.cpp; do
  bin="$test_dir/$(basename "$repro" .cpp)"
  g++ -std=c++17 -Wall -Wextra -Werror -pedantic -I "$inc" -I "$root/tests" "$repro" -o "$bin"
  "$bin"
done
# C11/C12 assert that old vulnerabilities exist. Preserve their exact bytes
# and replay each independently; expected vulnerability-assert aborts are
# diagnostic only. test_m1_r4_ack_isolation above proves positive safety.
python3 "$root/tests/replay_c11_c12.py"
python3 "$root/tests/adapter_compile_check.py"
if compgen -G "$root/tests/test_*.py" >/dev/null; then
  python3 -m unittest discover -s "$root/tests" -p 'test_*.py' -v
fi
