#!/usr/bin/env bash
# 仅用于开发机：真实ROS本机合成peer，禁止连接Agent/飞控；输出是模型数据。
# 参数1：已核验的阶段2 x86节点路径；参数2：新的结果目录。
set -eo pipefail
if [ "$#" -ne 2 ]; then
  echo "用法：bash $0 /absolute/path/fc01_system_node /new/output/directory" >&2
  exit 2
fi
fc01_stage3_node=$(realpath "$1")
fc01_stage3_output=$(realpath -m "$2")
fc01_stage3_repo=$(cd "$(dirname "$0")/../../.." && pwd)
if [ -e "$fc01_stage3_output" ]; then
  echo "结果目录已存在，请指定新目录以保留旧证据。" >&2
  exit 2
fi
mkdir -p "$fc01_stage3_output"
# Humble环境脚本可能引用未定义变量；不在source前启用nounset。
source /opt/ros/humble/setup.bash
source /home/jikl62/FC01_R632_P0_ATTITUDE_AUDIT_CANDIDATE/install_r632/px4_msgs/share/px4_msgs/local_setup.bash
fc01_stage3_install=$(cd "$(dirname "$fc01_stage3_node")/../../.." && pwd)
source "$fc01_stage3_install/local_setup.bash"
cd "$fc01_stage3_repo"
for fc01_stage3_scenario in normal range_unavailable feedback_stale quat_reset vertical_reset offboard_loss offboard_flag rc_takeover kill velocity_warning; do
  /usr/bin/python3 tests/ros_stage3_lifecycle.py --node "$fc01_stage3_node" \
    --output "$fc01_stage3_output/$fc01_stage3_scenario-replay" \
    --scenario "$fc01_stage3_scenario" --motion replay \
    > "$fc01_stage3_output/$fc01_stage3_scenario-replay.log" 2>&1
  tail -n 1 "$fc01_stage3_output/$fc01_stage3_scenario-replay.log"
done
/usr/bin/python3 tests/ros_stage3_lifecycle.py --node "$fc01_stage3_node" \
  --output "$fc01_stage3_output/normal-plant" --scenario normal --motion plant \
  > "$fc01_stage3_output/normal-plant.log" 2>&1
tail -n 1 "$fc01_stage3_output/normal-plant.log"
