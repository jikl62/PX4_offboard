#!/usr/bin/env bash
# 安装用户级 systemd 服务文件，设置先交权后退出的无限停止等待；此命令不自动 enable/start。
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$root/scripts/common.sh"
[[ "$FC01_BASE" == "$HOME/fc01_system" ]] || { echo '用户服务固定使用 ~/fc01_system；自定义路径请前台启动。' >&2; exit 2; }
mkdir -p "$HOME/.config/systemd/user"
# 服务载荷的 ExecStart/Restart/TimeoutStopSec 保持原值；在 shell 外层解释，避免修改生成的服务字节。
cat > "$HOME/.config/systemd/user/fc01-system.service" <<'UNIT'
[Unit]
Description=FC01 companion controller (idle at startup)
After=network-online.target

[Service]
Type=simple
ExecStart=/bin/bash %h/fc01_system/current/scripts/start.sh
Restart=no
TimeoutStopSec=infinity
KillMode=mixed

[Install]
WantedBy=default.target
UNIT
# 只让 systemd 重读定义；是否 enable/启动需操作者另行显式执行。
systemctl --user daemon-reload
echo '服务文件已安装。登录会话内启用：systemctl --user enable --now fc01-system.service'
echo '无桌面登录也要启动：先由你执行 sudo loginctl enable-linger "$USER"，再启用服务。'
