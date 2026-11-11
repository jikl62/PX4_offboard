#!/usr/bin/env bash
# 部署 CLI 的薄入口：严格 shell 选项只作用于本进程，离线/激活/回滚由 Python 合同决定。
# Strict handling stays inside this script, never in the operator's shell.
set -euo pipefail
fc01_deploy_entry=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
# 不在 shell 层隐式选 activate，参数原样交给 navigation_deploy.py。
exec /usr/bin/python3 "$fc01_deploy_entry/navigation_deploy.py" "$@"
