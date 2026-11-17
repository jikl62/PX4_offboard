#!/usr/bin/env bash
# 起飞推力斜坡/高度跟踪的合成模型验证；可选 sensitivity 独立输出已知低悬停能力失败。
set -euo pipefail
# 异常断言不留下 core 文件；这不改变测试退出码和失败判断。
ulimit -c 0
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
test_dir="$(mktemp -d)"
trap 'rm -rf -- "$test_dir"' EXIT
# 各场景分别编译执行，名义模型轨迹不计作现场飞行证据。
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
# 敏感性报告必须显式给输出路径，已知失败结果保留而非算成通过。
if [[ "${1:-}" == --sensitivity ]]; then
  if [[ -z "${2:-}" ]]; then
    echo 'usage: bash scripts/r632_ramp_guard_offline_verify.sh --sensitivity OUTPUT.csv' >&2
    exit 2
  fi
  "$test_dir/test_r632_takeoff_plant" sensitivity > "$2"
fi
echo 'R632_RAMP_GUARD_FOCUSED_OFFLINE_PASS (no ROS/PX4/hardware flight claim)'
