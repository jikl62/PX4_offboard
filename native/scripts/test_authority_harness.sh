#!/usr/bin/env bash
# 控制权时序离线驱动：把固定 harness 的观测输出到指定文件，或默认标准输出。
set -euo pipefail
ulimit -c 0
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
output="${1:-/dev/stdout}"
# 程序只由合成状态驱动，不访问真实 PX4；临时目录由本脚本退出时回收。
temporary="$(mktemp -d)"
trap 'rm -rf -- "$temporary"' EXIT
g++ -std=c++17 -Wall -Wextra -Werror -pedantic -I "$root/src/jikl_fc01_system/include" \
  "$root/tests/authority_harness.cpp" -o "$temporary/harness"
"$temporary/harness" > "$output"
