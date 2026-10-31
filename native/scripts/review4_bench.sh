#!/usr/bin/env bash
# Operator-run, fixed no-propeller bench only. Starting this node does not Arm;
# fcctl still requires ARM_ONLY and the separate BENCH ARM CONFIRM prompt.
set -eo pipefail
review4_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$review4_root"
review4_node="$review4_root/install_review4/jikl_fc01_system/lib/jikl_fc01_system/fc01_system_node"
if [[ ! -x "$review4_node" ]]; then
  echo 'STOP: Build Review4 with bash scripts/review4_build.sh first' >&2
  exit 1
fi
if ps -C fc01_system_node -o pid= | grep -q '[0-9]'; then
  echo 'STOP: An FC01 control node is already running; do not start a second one' >&2
  exit 1
fi
source /opt/ros/humble/setup.bash
source "$review4_root/install_review4/local_setup.bash"
review4_socket="$HOME/.cache/fc01/r62c_review4_$(date +%s)_$$.sock"
review4_check="$(python3 tools/socket_safety.py "$review4_socket")"
printf '%s\n' "$review4_check"
if [[ "$review4_check" != "SOCKET_ABSENT: $review4_socket" ]]; then
  echo 'STOP: Socket safety check failed' >&2
  exit 1
fi
printf '%s\n' "$review4_socket" > .review4_socket
mkdir -p logs
review4_log="$review4_root/logs/review4_bench_$(date +%Y%m%d_%H%M%S)_$$.log"
printf '%s\n' "$review4_log" > .review4_log
printf 'Review4: fixed airframe, all propellers removed, Disarmed and independent stop ready.\nLog: %s\n' "$review4_log"
"$review4_node" \
  --ros-args --params-file "$review4_root/config/attitude_altitude.yaml" \
  -p flight_enabled:=false -p flight_actions_enabled:=true \
  -p arm_enabled:=true -p takeoff_enabled:=false \
  -p flight_parameters_confirmed:=true -p bench_arm_only_enabled:=true \
  -p rc_takeover_confirmed:=true -p autonomous_failsafe_confirmed:=false \
  -p socket_path:="$review4_socket" \
  2>&1 | tee "$review4_log"
