#!/usr/bin/env bash
# Operator-run, fixed no-propeller bench only. Starting this node does not Arm;
# fcctl still requires ARM_ONLY and the separate BENCH ARM CONFIRM prompt.
set -eo pipefail
review3_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$review3_root"
review3_node="$review3_root/install_review3/jikl_fc01_system/lib/jikl_fc01_system/fc01_system_node"
if [[ ! -x "$review3_node" ]]; then
  echo 'STOP: Build Review3 with bash scripts/review3_build.sh first' >&2
  exit 1
fi
if ps -C fc01_system_node -o pid= | grep -q '[0-9]'; then
  echo 'STOP: An FC01 control node is already running; do not start a second one' >&2
  exit 1
fi
source /opt/ros/humble/setup.bash
source "$review3_root/install_review3/local_setup.bash"
review3_socket="$HOME/.cache/fc01/r62c_review3_$(date +%s)_$$.sock"
review3_check="$(python3 tools/socket_safety.py "$review3_socket")"
printf '%s\n' "$review3_check"
if [[ "$review3_check" != "SOCKET_ABSENT: $review3_socket" ]]; then
  echo 'STOP: Socket safety check failed' >&2
  exit 1
fi
printf '%s\n' "$review3_socket" > .review3_socket
mkdir -p logs
review3_log="$review3_root/logs/review3_bench_$(date +%Y%m%d_%H%M%S)_$$.log"
printf '%s\n' "$review3_log" > .review3_log
printf 'Review3: fixed airframe, all propellers removed, Disarmed and independent stop ready.\nLog: %s\n' "$review3_log"
"$review3_node" \
  --ros-args --params-file "$review3_root/config/attitude_altitude.yaml" \
  -p flight_enabled:=false -p flight_actions_enabled:=true \
  -p arm_enabled:=true -p takeoff_enabled:=false \
  -p flight_parameters_confirmed:=true -p bench_arm_only_enabled:=true \
  -p rc_takeover_confirmed:=true -p autonomous_failsafe_confirmed:=false \
  -p socket_path:="$review3_socket" \
  2>&1 | tee "$review3_log"
