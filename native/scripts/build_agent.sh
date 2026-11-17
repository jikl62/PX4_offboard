#!/usr/bin/env bash
# 独立构建 Micro-XRCE-DDS-Agent v2.4.3；只在缺少源码时下载，不启动 Agent 或飞控节点。
set -euo pipefail
base="${FC01_HOME:-$HOME/fc01_system}"
source_dir="$base/deps-src/Micro-XRCE-DDS-Agent"
mkdir -p "$base/deps-src"
if [[ ! -d "$source_dir" ]]; then git clone --depth 1 --branch v2.4.3 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git "$source_dir"; fi
# 源码必须精确位于指定 tag 且受跟踪文件未修改，避免静默构建未知协议实现。
[[ "$(git -C "$source_dir" describe --tags --exact-match)" == v2.4.3 && -z "$(git -C "$source_dir" status --porcelain --untracked-files=no)" ]] || { echo 'Agent 源码版本/修改不符合 v2.4.3。' >&2; exit 2; }
# 编译/安装均指向 FC01 私有依赖目录，单任务减少目标板内存压力。
cmake -S "$source_dir" -B "$base/agent-build" -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$base/deps"
cmake --build "$base/agent-build" --parallel 1
cmake --install "$base/agent-build"
echo "Agent 已原生编译到 $base/deps/bin/MicroXRCEAgent"
echo 'agent.sh 自动加入 deps/lib 与 deps/lib64 动态库路径。'
