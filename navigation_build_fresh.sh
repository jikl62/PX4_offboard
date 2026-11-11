#!/usr/bin/env bash
# 原生离线构建入口：生成独立日志和阶段标签，不启动控制器、Agent、探测或 systemd。
# Native offline build/reuse only: no controller, Agent, probe or systemd action.
set -Eeuo pipefail
fc01_build_entry=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
mkdir -p "$HOME/FC01_navigation"
fc01_build_log=$(mktemp "$HOME/FC01_navigation/offline-build-$(date -u +%Y%m%dT%H%M%SZ)-XXXXXX.log")
export FC01_BUILD_LOG="$fc01_build_log"
# 把标准输出/错误同时写入终端和专用日志，ERR trap 保留失败阶段及原退出码。
exec > >(tee -a "$fc01_build_log") 2>&1
trap 'fc01_build_status=$?; echo "FC01_OFFLINE_BUILD_FAILED stage=${fc01_build_stage:-unknown} exit=$fc01_build_status log=$fc01_build_log" >&2; exit "$fc01_build_status"' ERR
printf 'FC01_OFFLINE_BUILD_LOG=%s\n' "$fc01_build_log"
# 先校验 ROS/SDK 环境再构建，避免把来源不明的依赖产物当成当前源码证据。
fc01_build_stage=environment
source "$fc01_build_entry/navigation_environment.sh"
fc01_load_environment "$fc01_build_entry"
fc01_build_stage=build
# build_release 校验目标架构、源码/依赖摘要和安装产物，复用失败会保留原目录。
/usr/bin/python3 "$fc01_build_entry/navigation_build_evidence.py" build --entry "$fc01_build_entry"
