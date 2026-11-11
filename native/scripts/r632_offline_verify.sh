#!/usr/bin/env bash
# 有界离地推力储备的离线组合验证：先独立编译专项，再串联已有候选与斜坡保护回归。
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
bin="$(mktemp)"
# 只清理当前 mktemp 程序，测试失败由严格退出传播，不重试或改参数。
trap 'rm -f -- "$bin"' EXIT
g++ -std=c++17 -Wall -Wextra -Werror -pedantic \
  -I "$root/src/jikl_fc01_system/include" \
  "$root/tests/test_r632_bounded_liftoff_reserve.cpp" -o "$bin"
"$bin"
bash "$root/scripts/r63_offline_verify.sh"
bash "$root/scripts/r632_ramp_guard_offline_verify.sh"
