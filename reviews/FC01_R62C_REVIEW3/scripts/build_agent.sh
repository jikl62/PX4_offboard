#!/usr/bin/env bash
set -euo pipefail
base="${FC01_HOME:-$HOME/fc01_system}"
source_dir="$base/deps-src/Micro-XRCE-DDS-Agent"
mkdir -p "$base/deps-src"
if [[ ! -d "$source_dir" ]]; then git clone --depth 1 --branch v2.4.3 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git "$source_dir"; fi
[[ "$(git -C "$source_dir" describe --tags --exact-match)" == v2.4.3 && -z "$(git -C "$source_dir" status --porcelain --untracked-files=no)" ]] || { echo 'Agent 源码版本/修改不符合 v2.4.3。' >&2; exit 2; }
cmake -S "$source_dir" -B "$base/agent-build" -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$base/deps"
cmake --build "$base/agent-build" --parallel 1
cmake --install "$base/agent-build"
echo "Agent 已原生编译到 $base/deps/bin/MicroXRCEAgent"
echo 'agent.sh 自动加入 deps/lib 与 deps/lib64 动态库路径。'
