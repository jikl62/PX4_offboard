#!/usr/bin/env python3
"""Configure the already-built navigation service; never start it or an Agent."""
from __future__ import annotations

import argparse
import fcntl
import hashlib
import json
import os
from pathlib import Path
import pwd
import subprocess
import tempfile
from datetime import datetime

from launcher import ControlError, preflight, private_dir, process_list
from navigation_install import (MARKER, OLD, SERVICE, boot, managed_unit,
                                render_unit, unit_info)
from navigation_service import check_controller

ENTRY = Path(__file__).resolve().parent
UNIT_DIR = Path('/etc/systemd/system')


def stopped_services(units, run):
    """No stop/kill is implicit: stop any old starter before configuring."""
    info = {name: unit_info(name, run) for name in (SERVICE, OLD)}
    for name, row in info.items():
        if row.get('MainPID', '0') != '0' or row.get('ActiveState') not in ('inactive', 'failed', ''):
            raise ControlError('服务仍在运行，先在接地上锁后人工停止：'+name)
        if row.get('DropInPaths'):
            raise ControlError('存在服务覆盖配置，保留它并拒绝变更：'+name)
    if info[OLD].get('UnitFileState') in ('enabled', 'enabled-runtime', 'linked', 'linked-runtime'):
        raise ControlError('旧待命服务仍有开机入口；接地上锁后人工确认并执行：sudo systemctl disable '+OLD)
    if any(Path(p.argv[0]).name == 'fc01_system_node' for p in process_list()):
        raise ControlError('已有控制器运行；保留它，拒绝配置第二个入口')
    return info


def existing_agent(entry, home, project):
    binary = check_controller(entry=entry)
    if not binary.resolve().is_relative_to(entry.resolve()):
        raise ControlError('新版控制器指向其他目录，拒绝启动旧版二进制')
    _, agent = preflight(project, home/'fc01_agent_build_offline/MicroXRCEAgent', '/dev/ttyS2',
                         controller=binary, config_path=entry/'native/config/flight_test.yaml')
    if agent is None:
        raise ControlError('未找到可复用的既有 Agent；本工具不启动或停止 Agent')
    return binary, agent


def configure(entry, home, project, *, units=UNIT_DIR, run=subprocess.run):
    info = stopped_services(units, run)
    path = units/SERVICE
    managed_unit(path, MARKER, info[SERVICE])
    binary, agent = existing_agent(entry, home, project)
    folder = home/'FC01_navigation'
    private_dir(folder)
    config = folder/'restore-flight-test-runtime.yaml'
    boot(entry, home, project, 'prepare-config', '--mode', 'flight-test',
         '--confirm-flight-test', '--file', config)
    boot(entry, home, project, 'check-config', '--mode', 'flight-test',
         '--confirm-flight-test', '--file', config, '--prefix', entry/'install')
    user = pwd.getpwuid(os.getuid()).pw_name
    # The preparation command is manual. Do not enable at boot or retry a
    # failed controller automatically; RC action remains a separate decision.
    unit = render_unit(user, home, entry, project, 'flight-test', True)
    unit = unit.replace('Restart=on-failure', 'Restart=no')
    with tempfile.TemporaryDirectory(prefix='fc01_restore_unit_') as temp:
        staged = Path(temp)/SERVICE
        staged.write_text(unit)
        run(['/usr/bin/systemd-analyze', 'verify', str(staged)], check=True)
        run(['sudo', '-v'], check=True)
        # Password entry must precede new, passive ground evidence.
        boot(entry, home, project, 'probe', '--file', folder/'probe.json')
        from navigation_install import ground_permit_fresh
        if not ground_permit_fresh(home):
            raise ControlError('缺少新鲜未解锁接地证据，未变更服务')
        stopped_services(units, run)
        checked_binary, checked_agent = existing_agent(entry, home, project)
        if checked_binary != binary or checked_agent != agent:
            raise ControlError('校验后控制器或 Agent 已变化，未变更服务')
        managed_unit(path, MARKER, unit_info(SERVICE, run))
        backups = folder/'service-backups'
        private_dir(backups)
        backup = backups/datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        backup.mkdir(mode=0o700)
        present = path.is_file()
        original = path.read_bytes() if present else None
        if present:
            (backup/SERVICE).write_bytes(original)
        (backup/'manifest.json').write_text(json.dumps({
            'service': SERVICE, 'original_present': present, 'entry': str(entry),
            'project': str(project), 'unit_sha256': hashlib.sha256(unit.encode()).hexdigest(),
            'original_unit_file_state': info[SERVICE].get('UnitFileState', ''),
            'original_sha256': hashlib.sha256(original).hexdigest() if present else None,
            'controller': str(binary.resolve()),
            'controller_sha256': hashlib.sha256(binary.read_bytes()).hexdigest(),
            'agent_pid': agent.pid,
        }, indent=2)+'\n')
        if not ground_permit_fresh(home):
            raise ControlError('接地证据已过期，未变更服务')
        # A previously enabled unit must not turn the Flight Test profile on
        # after reboot.  Disable only the inactive, package-owned unit; this
        # does not stop a process and the original state is recorded below.
        if info[SERVICE].get('UnitFileState') in ('enabled', 'enabled-runtime', 'linked', 'linked-runtime'):
            run(['sudo', '-n', '/usr/bin/systemctl', 'disable', SERVICE], check=True)
        # Does not enable, start, stop, kill, or replace any binary.
        run(['sudo', '-n', '/usr/bin/install', '-m', '0644', str(staged), str(path)], check=True)
        run(['sudo', '-n', '/usr/bin/systemctl', 'daemon-reload'], check=True)
    print('FC01_NEW_SERVICE_CONFIGURED controller='+str(binary.resolve()), flush=True)
    print('FC01_EXISTING_AGENT_RETAINED pid='+str(agent.pid), flush=True)
    print('FC01_SERVICE_NOT_STARTED boot_enabled=disabled original_enabled_state='+info[SERVICE].get('UnitFileState', '')+
          ' rollback='+str(backup), flush=True)
    return backup


def rollback(entry, home, project, backup, *, units=UNIT_DIR, run=subprocess.run):
    backup = backup.expanduser().resolve()
    allowed = (home/'FC01_navigation/service-backups').resolve()
    if not backup.is_relative_to(allowed) or backup == allowed:
        raise ControlError('回滚记录必须位于当前用户 service-backups 目录')
    manifest = json.loads((backup/'manifest.json').read_text())
    if manifest.get('service') != SERVICE:
        raise ControlError('回滚记录服务名不匹配')
    stopped_services(units, run)
    path = units/SERVICE
    managed_unit(path, MARKER, unit_info(SERVICE, run))
    if not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest() != manifest.get('unit_sha256'):
        raise ControlError('当前服务已被修改，保留它；拒绝覆盖其他变更')
    restored = backup/SERVICE
    if manifest.get('original_present') is True:
        if (restored.is_symlink() or not restored.is_file() or
                not restored.read_text().startswith(MARKER+'\n') or
                hashlib.sha256(restored.read_bytes()).hexdigest() != manifest.get('original_sha256')):
            raise ControlError('原服务备份无效，未回滚')
    run(['sudo', '-v'], check=True)
    boot(entry, home, project, 'probe', '--file', home/'FC01_navigation/probe.json')
    from navigation_install import ground_permit_fresh
    if not ground_permit_fresh(home):
        raise ControlError('缺少新鲜未解锁接地证据，未回滚')
    stopped_services(units, run)
    if manifest.get('original_present') is True:
        run(['sudo', '-n', '/usr/bin/install', '-m', '0644', str(restored), str(path)], check=True)
    else:
        run(['sudo', '-n', '/usr/bin/rm', '--', str(path)], check=True)
    run(['sudo', '-n', '/usr/bin/systemctl', 'daemon-reload'], check=True)
    print('FC01_SERVICE_ROLLED_BACK_NOT_STARTED; 原服务保持停止，需现场人工选择是否启动', flush=True)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=('configure', 'rollback'))
    parser.add_argument('--project-root', type=Path, required=True,
                        help='现有 ROS/px4_msgs SDK 根目录；不作为控制器或参数来源')
    parser.add_argument('--confirm-flight-test', action='store_true')
    parser.add_argument('--backup', type=Path)
    args = parser.parse_args(argv)
    if os.getuid() == 0:
        parser.error('请以 orangepi 普通用户运行；只在写服务时使用 sudo')
    if args.action == 'configure' and not args.confirm_flight_test:
        parser.error('configure 必须显式提供 --confirm-flight-test')
    if args.action == 'rollback' and args.backup is None:
        parser.error('rollback requires --backup')
    os.umask(0o077)
    home = Path(pwd.getpwuid(os.getuid()).pw_dir)
    project = args.project_root.expanduser().resolve()
    # navigation_restore.sh supplies the reviewed ROS/px4_msgs environment.
    os.environ['FC01_PROJECT_ROOT'] = str(project)
    private_dir(home/'.cache/fc01_navigation')
    try:
        with open(home/'.cache/fc01_navigation/install.lock', 'a') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            if args.action == 'configure':
                configure(ENTRY, home, project)
            else:
                rollback(ENTRY, home, project, args.backup)
    except (OSError, ValueError, RuntimeError, subprocess.SubprocessError) as exc:
        print('FC01_SERVICE_UNCHANGED_OR_STOPPED: '+str(exc), flush=True)
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
