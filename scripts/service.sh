#!/usr/bin/env bash
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$root/scripts/common.sh"
[[ "$FC01_BASE" == "$HOME/fc01_system" ]] || { echo '用户服务固定使用 ~/fc01_system；自定义路径请前台启动。' >&2; exit 2; }
mkdir -p "$HOME/.config/systemd/user"
cat > "$HOME/.config/systemd/user/fc01-system.service" <<'UNIT'
[Unit]
Description=FC01 companion controller (idle at startup)
After=network-online.target

[Service]
Type=simple
ExecStart=/bin/bash %h/fc01_system/current/scripts/start.sh
Restart=no
TimeoutStopSec=75
KillMode=mixed

[Install]
WantedBy=default.target
UNIT
systemctl --user daemon-reload
echo '服务文件已安装。登录会话内启用：systemctl --user enable --now fc01-system.service'
echo '无桌面登录也要启动：先由你执行 sudo loginctl enable-linger "$USER"，再启用服务。'
