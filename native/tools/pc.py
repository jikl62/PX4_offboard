#!/usr/bin/env python3
# PC 端控制台转接器：通过用户提供的 SSH 主机运行板载 console.sh，不自行实现第二套飞控协议。
"""Run the onboard operator console over authenticated OpenSSH."""
import argparse
import re
import shlex
import subprocess
import sys


# 主机名按白名单验证并置于 -- 后；控制台参数用 shlex 转义，避免被当成 SSH 选项或远端命令。
def ssh_args(host, console_args):
    if not re.fullmatch(r'[A-Za-z0-9_][A-Za-z0-9_.@:\[\]-]*', host) or host.startswith('-'):
        raise ValueError('请输入 SSH 用户@IP/主机名，不包含 SSH 选项')
    # 远端使用板载 current 版本；交互模式申请终端，单条命令模式关闭伪终端。
    command = 'exec bash "$HOME/fc01_system/current/scripts/console.sh"'
    if console_args:
        command += ' ' + shlex.join(console_args)
    return [
        'ssh',
        '-T' if '--command' in console_args else '-tt',
        '--',
        host,
        'bash -c ' + shlex.quote(command),
    ]


# 把命令和 JSON 选项传给远端控制台；返回 SSH 退出码，输入或进程错误返回 2。
def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('host')
    parser.add_argument('--command')
    parser.add_argument('--json', action='store_true')
    args = parser.parse_args()
    options = []
    if args.command:
        options += ['--command', args.command]
    if args.json:
        options += ['--json']
    try:
        return subprocess.call(ssh_args(args.host, options))
    except (ValueError, OSError) as error:
        print(str(error), file=sys.stderr)
        return 2


if __name__ == '__main__':
    sys.exit(main())
