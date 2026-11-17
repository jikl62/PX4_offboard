#!/usr/bin/env bash
# 部署环境共享函数：SDK 选择优先级为显式环境、部署配置、既有默认目录；路径/环境不完整即拒绝。
# Source this helper; changes apply only to the calling deployment process.
# 返回经过真实路径解析且具有 install_r632/local_setup.bash 的 SDK；子 Python 仅输出所选路径。
fc01_resolve_sdk() {
  local fc01_env_entry="$1" fc01_env_sdk
  fc01_env_sdk=$(/usr/bin/python3 - "$fc01_env_entry" "${FC01_PROJECT_ROOT:-}" <<'PY'
import json
from pathlib import Path
import sys
entry=Path(sys.argv[1]).resolve()
explicit=sys.argv[2]
config=entry/'deployment-config.json'
if explicit:
    selected=Path(explicit).expanduser()
elif config.exists():
    data=json.loads(config.read_text())
    value=data.get('sdk_root') or data.get('project_root') or data.get('FC01_PROJECT_ROOT')
    if not isinstance(value,str) or not value:
        raise SystemExit('FC01_SDK_ENVIRONMENT_FAILED: deployment-config.json lacks sdk_root')
    selected=Path(value).expanduser()
else:
    selected=Path.home()/'FC01_ARM64_PORT_20261020/FC01_R632_P0_ATTITUDE_AUDIT_CANDIDATE'
if not selected.is_absolute():
    raise SystemExit('FC01_SDK_ENVIRONMENT_FAILED: SDK path must be absolute')
selected=selected.resolve()
if not (selected/'install_r632/local_setup.bash').is_file():
    raise SystemExit('FC01_SDK_ENVIRONMENT_FAILED: missing SDK install_r632/local_setup.bash: '+str(selected))
print(selected)
PY
  ) || return $?
  export FC01_PROJECT_ROOT="$fc01_env_sdk"
}

# 保存调用方 nounset 状态，临时允许 ROS setup 读取未设置变量，结束时恢复原选项。
fc01_load_environment() {
  local fc01_env_entry="$1" fc01_env_nounset=0 fc01_env_result=0
  fc01_resolve_sdk "$fc01_env_entry" || return $?
  if [[ ! -f /opt/ros/humble/setup.bash ]]; then
    echo 'FC01_SDK_ENVIRONMENT_FAILED: ROS 2 Humble setup is missing' >&2
    return 2
  fi
  # 不能假定调用方已关闭 -u；ROS/SDK source 失败应保留退出码并拒绝继续。
  case "$-" in *u*) fc01_env_nounset=1 ;; esac
  set +u
  source /opt/ros/humble/setup.bash || fc01_env_result=$?
  if [[ "$fc01_env_result" == 0 ]]; then
    # 此处加载所选 SDK 的依赖环境；它本身不证明当前源码生成的 FC01 消息或适配器已安装。
    # 独立验证新视觉接口时，应另行加载对应构建的 overlay 并核对模块实际路径。
    source "$FC01_PROJECT_ROOT/install_r632/local_setup.bash" || fc01_env_result=$?
  fi
  if [[ "$fc01_env_nounset" == 1 ]]; then set -u; fi
  if [[ "$fc01_env_result" != 0 ]]; then
    echo 'FC01_SDK_ENVIRONMENT_FAILED: sourcing ROS/SDK failed' >&2
    return "$fc01_env_result"
  fi
  if [[ "${ROS_DISTRO:-}" != humble ]]; then
    echo 'FC01_SDK_ENVIRONMENT_FAILED: ROS_DISTRO must be humble' >&2
    return 2
  fi
  # 正式链路使用域 0 和网络可见 DDS；此设置是现有部署合同，不代表已证明带宽或来源正确。
  export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 PYTHONUNBUFFERED=1
}
