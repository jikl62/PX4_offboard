#!/usr/bin/env bash
set -euo pipefail
ulimit -c 0
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
output="${1:-/dev/stdout}"
temporary="$(mktemp -d)"
trap 'rm -rf -- "$temporary"' EXIT
g++ -std=c++17 -Wall -Wextra -Werror -pedantic -I "$root/src/jikl_fc01_system/include" \
  "$root/tests/authority_harness.cpp" -o "$temporary/harness"
"$temporary/harness" > "$output"
