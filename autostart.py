#!/usr/bin/env python3
# 历史关闭动作门待机服务的安装与管理CLI：源自autostart独立包，与正式导航服务分开识别。
# 安装会显式sudo写systemd并enable；状态/报告只读，服务启动本身不自动解锁或起飞。
"""Install/manage FC01 closed-gate standby at boot."""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import pwd
import re
import subprocess
import tempfile
import time
from datetime import datetime
import zipfile

from launcher import PROJECT

SERVICE = 'fc01-standby.service'
UNIT = Path('/etc/systemd/system') / SERVICE
MARKER = '# FC01_AUTOSTART_CLOSED_GATES_20261027_V1'


# 按systemd的值/ExecStart转义规则处理路径；拒绝控制字符，不能用shell转义代替unit转义。
def quote(value, executable=False):
    text = str(value)
    if any(x in text for x in ('\0', '\r', '\n')):
        raise ValueError('服务参数包含非法换行/空字符')
    text = text.replace('\\', '\\\\').replace('"', '\\"').replace('%', '%%')
    if executable:
        text = text.replace('$', '$$')
    return '"' + text + '"'


# 验证普通用户名和绝对entry，生成受MARKER标记的服务；信号交给监督器且禁止强杀恢复中的节点。
def render_unit(user, home, entry, project):
    if not re.fullmatch(r'[a-zA-Z_][a-zA-Z0-9_-]*\$?', user):
        raise ValueError('不支持此系统用户名')
    quote(entry)  # Validate controls; WorkingDirectory is a path, not a word list.
    if not entry.is_absolute():
        raise ValueError('服务目录必须为绝对路径')
    return f'''{MARKER}
[Unit]
Description=FC01 Orange Pi closed-gate standby
After=local-fs.target
StartLimitIntervalSec=0

[Service]
Type=simple
User={user}
WorkingDirectory={str(entry).replace('%', '%%')}
Environment={quote('HOME='+str(home))}
Environment={quote('FC01_PROJECT_ROOT='+str(project))}
Environment=ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 PYTHONUNBUFFERED=1
ExecStart=/bin/bash {quote(entry/'standby.sh', executable=True)}
StandardInput=null
StandardOutput=journal
StandardError=journal
SyslogIdentifier=fc01-standby
UMask=0077
NoNewPrivileges=true
Restart=on-failure
RestartSec=30s
# Supervisor owns native graceful stop; retained Agent must survive/re-enter.
KillMode=process
KillSignal=SIGTERM
SendSIGKILL=no
TimeoutStopSec=infinity

[Install]
WantedBy=multi-user.target
'''


# 只接受本包相同unit，保留外来/符号链接/不同目录服务；先systemd-analyze验证再安装。
def install_service(unit, destination=UNIT, run=subprocess.run):
    if destination.is_symlink():
        raise RuntimeError('同名服务是符号链接，已保留：' + str(destination))
    if destination.exists():
        old = destination.read_text(encoding='utf8')
        if not old.startswith(MARKER + '\n'):
            raise RuntimeError('同名服务不属于本包，已保留：' + str(destination))
        if old != unit:
            raise RuntimeError('已安装服务指向另一个目录；请先停用旧版本，未覆盖')
    else:
        with tempfile.TemporaryDirectory(prefix='fc01_unit_') as d:
            path = Path(d) / SERVICE
            path.write_text(unit, encoding='utf8')
            run(['/usr/bin/systemd-analyze', 'verify', str(path)], check=True)
            run(['sudo', '/usr/bin/install', '-m', '0644', str(path), str(destination)], check=True)
    run(['sudo', '/usr/bin/systemctl', 'daemon-reload'], check=True)
    run(['sudo', '/usr/bin/systemctl', 'enable', '--now', SERVICE], check=True)


# 读取服务与待机快照；墙钟年龄超限显示STALE，不能把旧ready当本次飞机状态。
def status(home):
    for op in ('is-enabled', 'is-active'):
        result = subprocess.run(['/usr/bin/systemctl', op, SERVICE], capture_output=True, text=True)
        print(op + ': ' + (result.stdout.strip() or result.stderr.strip()))
    path = home / 'FC01_autostart/status.json'
    if path.is_file():
        row = json.loads(path.read_text())
        age = time.time() - row.get('updated_unix', 0)
        if not 0 <= age <= 12:
            row = {**row, 'status': 'STALE', 'reason': '状态快照已过期；使用 logs 查看后台运行情况'}
        print('后台：' + row['status'] + '；' + row['reason'])
        issues = row.get('readiness_issues')
        if issues:
            print('反馈条件：' + ', '.join(issues))
    else:
        print('后台状态尚未生成，使用 logs 查看启动原因。')
    print('后台动作门关闭；CH6 导航触发将在下一阶段接入。')


# 收集已存在待机文件与只读服务查询到新ZIP；不触发探针或控制动作。
def report(home):
    folder = home / 'FC01_autostart'
    if not folder.is_dir():
        raise RuntimeError('没有后台运行记录')
    target = home / (
        'FC01_autostart_report_' + datetime.now().strftime('%Y%m%d_%H%M%S_%f') + '.zip'
    )
    with zipfile.ZipFile(target, 'x', zipfile.ZIP_DEFLATED) as z:
        for name in (
            'status.json',
            'runtime.yaml',
            'standby.log',
            'standby.log.1',
            'standby.log.2',
            'controller.log',
            'controller.log.1',
            'controller.log.2',
        ):
            path = folder / name
            if path.is_file() and not path.is_symlink():
                z.write(path, name)
        for name, argv in (
            ('service.txt', ['/usr/bin/systemctl', 'status', '--no-pager', SERVICE]),
            ('journal.txt', ['/usr/bin/journalctl', '-u', SERVICE, '-n', '100', '--no-pager']),
        ):
            p = subprocess.run(argv, capture_output=True, text=True, timeout=15)
            z.writestr(name, (p.stdout + p.stderr)[-262144:])
    target.chmod(0o600)
    print('自动报告：' + str(target))
    return target


# 默认安装仍需普通用户并先--validate环境；管理子命令显式调用systemctl，失败报告非零。
def main(argv=None):
    parser = argparse.ArgumentParser(description='FC01 开机后台待命；不自动解锁或起飞')
    parser.add_argument(
        'action',
        nargs='?',
        default='install',
        choices=('install', 'status', 'logs', 'report', 'start', 'stop', 'disable'),
    )
    args = parser.parse_args(argv)
    os.umask(0o077)
    account = pwd.getpwuid(os.getuid())
    home = Path(account.pw_dir)
    entry = Path(__file__).resolve().parent
    try:
        if args.action == 'install':
            if os.getuid() == 0:
                raise RuntimeError('请用 orangepi 普通用户运行；脚本会在安装服务时请求 sudo')
            project = (
                Path(os.environ.get('FC01_PROJECT_ROOT', str(home / PROJECT)))
                .expanduser()
                .resolve()
            )
            subprocess.run(
                ['/bin/bash', str(entry / 'standby.sh'), '--validate'],
                check=True,
                env={**os.environ, 'HOME': str(home), 'FC01_PROJECT_ROOT': str(project)},
                timeout=30,
            )
            install_service(render_unit(account.pw_name, home, entry, project))
            print('开机后台待命已启用。可关闭终端；启动不依赖桌面登录。')
            status(home)
        elif args.action == 'status':
            status(home)
        elif args.action == 'logs':
            subprocess.run(
                ['/usr/bin/journalctl', '-u', SERVICE, '-n', '60', '--no-pager'], check=False
            )
            for name in ('standby.log', 'controller.log'):
                p = home / 'FC01_autostart' / name
                if p.is_file():
                    print('\n' + name + ':', flush=True)
                    with open(p, 'rb') as f:
                        f.seek(max(0, p.stat().st_size - 12288))
                        print(f.read().decode(errors='replace'))
        elif args.action == 'report':
            report(home)
        else:
            op = {'start': ['start'], 'stop': ['stop'], 'disable': ['disable', '--now']}[
                args.action
            ]
            subprocess.run(['sudo', '/usr/bin/systemctl', *op, SERVICE], check=True)
            status(home)
        return 0
    except (OSError, RuntimeError, ValueError, subprocess.SubprocessError) as exc:
        print('未完成：' + str(exc), flush=True)
        print('查看诊断：bash ' + str(entry / 'autostart.sh') + ' logs', flush=True)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
