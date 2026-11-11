#!/usr/bin/env bash
# 原生脚本共享环境：可读取用户 settings.env，统一 DDS 域、控制 socket、串口设备与波特率。
FC01_BASE="${FC01_HOME:-$HOME/fc01_system}"
if [[ -f "$FC01_BASE/settings.env" ]]; then source "$FC01_BASE/settings.env"; fi
export ROS_DOMAIN_ID="${FC01_DOMAIN:-0}"
export ROS_LOCALHOST_ONLY=0
# 本地控制通道是 Unix socket；设备默认值是稳定 by-id 路径，波特率单位 bit/s。
export FC01_SOCKET="${FC01_SOCKET:-$HOME/.cache/fc01/control.sock}"
export FC01_DEVICE="${FC01_DEVICE:-/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0}"
export FC01_BAUD="${FC01_BAUD:-921600}"
