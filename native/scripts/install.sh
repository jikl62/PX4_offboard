#!/usr/bin/env bash
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ "${FC01_CLEAN_BUILD:-0}" != 1 ]]; then
  exec env -u AMENT_PREFIX_PATH -u COLCON_PREFIX_PATH -u CMAKE_PREFIX_PATH -u PYTHONPATH -u LD_LIBRARY_PATH -u ROS_PACKAGE_PATH -u ROS_DISTRO FC01_CLEAN_BUILD=1 bash --noprofile --norc "$root/scripts/install.sh" "$@"
fi
python3 "$root/tools/deploy.py" check-platform
ros_setup="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
[[ -f "$ros_setup" ]] || { echo '缺少 ROS Humble；先按 README 准备依赖。' >&2; exit 2; }
command -v colcon >/dev/null || { echo '缺少 colcon。' >&2; exit 2; }
messages="${1:-${FC01_MSGS_SOURCE:-$HOME/fc01_real_ws/src/px4_msgs}}"
if [[ $# == 0 && -d "$root/vendor/px4_msgs" ]]; then messages="$root/vendor/px4_msgs"; fi
python3 "$root/tools/deploy.py" verify-messages "$messages"
bash "$root/scripts/test.sh"
base="${FC01_HOME:-$HOME/fc01_system}"
mkdir -p "$base/releases"
version="$(cat "$root/VERSION")"
[[ "$version" =~ ^[A-Za-z0-9][A-Za-z0-9._-]*$ ]] || { echo 'VERSION invalid' >&2; exit 2; }
release="$(mktemp -d "$base/releases/$version-$(date -u +%Y%m%dT%H%M%SZ)-XXXXXX")"
python3 - "$root" "$release" <<'PY'
import shutil,sys
from pathlib import Path
source,target=map(Path,sys.argv[1:])
for name in ('src','scripts','tools','config','docs','tests','LICENSE','README_zh.md','VERSION'):
    item=source/name
    if item.is_dir():shutil.copytree(item,target/name,ignore=shutil.ignore_patterns('__pycache__','*.pyc'))
    elif item.is_file():shutil.copy2(item,target/name)
PY
mkdir -p "$release/workspace/src"
cp -a "$messages" "$release/workspace/src/px4_msgs"
cp -a "$release/src/jikl_fc01_system" "$release/workspace/src/jikl_fc01_system"
set +u
source "$ros_setup"
set -u
cd "$release/workspace"
colcon build --parallel-workers 1 --packages-select px4_msgs jikl_fc01_system --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
[[ -x install/jikl_fc01_system/lib/jikl_fc01_system/fc01_system_node ]] || { echo '缺少编译输出，不切换版本。' >&2; exit 2; }
printf '%s\n' "$release" > "$base/built-release"
echo "新版本已构建：$release"
echo "当前版本未切换（包括首次安装）。核对并地面静置后显式执行：bash $release/scripts/update.sh $release"
