#!/usr/bin/env python3
"""Run the onboard operator console over authenticated OpenSSH."""
import argparse
import re
import shlex
import subprocess
import sys

def ssh_args(host,console_args):
    if not re.fullmatch(r'[A-Za-z0-9_][A-Za-z0-9_.@:\[\]-]*',host) or host.startswith('-'):
        raise ValueError('请输入 SSH 用户@IP/主机名，不包含 SSH 选项')
    command='exec bash "$HOME/fc01_system/current/scripts/console.sh"'
    if console_args:
        command+=' '+shlex.join(console_args)
    return ['ssh','-T' if '--command' in console_args else '-tt','--',host,'bash -c '+shlex.quote(command)]

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('host')
    parser.add_argument('--command')
    parser.add_argument('--json',action='store_true')
    args=parser.parse_args()
    options=[]
    if args.command:
        options+=['--command',args.command]
    if args.json:
        options+=['--json']
    try:
        return subprocess.call(ssh_args(args.host,options))
    except (ValueError,OSError) as error:
        print(str(error),file=sys.stderr);return 2

if __name__=='__main__':
    sys.exit(main())
