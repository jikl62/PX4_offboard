#!/usr/bin/env bash
# Source this helper; changes apply only to the calling deployment process.
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

fc01_load_environment() {
  local fc01_env_entry="$1" fc01_env_nounset=0 fc01_env_result=0
  fc01_resolve_sdk "$fc01_env_entry" || return $?
  if [[ ! -f /opt/ros/humble/setup.bash ]]; then
    echo 'FC01_SDK_ENVIRONMENT_FAILED: ROS 2 Humble setup is missing' >&2
    return 2
  fi
  case "$-" in *u*) fc01_env_nounset=1 ;; esac
  set +u
  source /opt/ros/humble/setup.bash || fc01_env_result=$?
  if [[ "$fc01_env_result" == 0 ]]; then
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
  export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 PYTHONUNBUFFERED=1
}
