#!/usr/bin/env python3
# 正式部署事务入口：离线构建/校验和在线激活分离，默认offline；源码摘要不符就保留既有运行服务。
# 真正服务切换与回滚由navigation_restore持完整事务锁、地面探针和恢复逻辑执行。
"""Separate offline ARM64 preparation from explicitly selected online activation."""
from __future__ import annotations

import argparse
from collections import deque
from datetime import datetime, timezone
import fcntl
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import pwd
import subprocess
import sys
import tempfile

ENTRY = Path(__file__).resolve().parent


# 部署前提不满足的可报告异常；不会被解释为自动继续或回退到未知参数。
class DeploymentError(RuntimeError):
    pass


# 用实际文件字节计算摘要，不能只比较mtime或旧文件名。
def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


# 在目标同目录私有临时文件写完后原子替换；finally仅清本操作残留临时路径。
def write_json(path, value):
    path = Path(path)
    path.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
    fd, temp = tempfile.mkstemp(prefix='.' + path.name, dir=path.parent)
    try:
        with os.fdopen(fd, 'w') as output:
            json.dump(value, output, ensure_ascii=False, indent=2)
            output.write('\n')
        os.replace(temp, path)
    finally:
        if os.path.exists(temp):
            os.unlink(temp)


# 逐项拒绝绝对/穿越/符号链接/越界/缺失/摘要不同的载荷；返回通过的当前清单。
def verify_payload(entry):
    manifest = json.loads((entry / 'FC01_RELEASE.json').read_text())
    files = manifest.get('payload_sha256')
    if not isinstance(files, dict) or not files:
        raise DeploymentError('安装包缺少完整源码清单，未继续部署')
    for name, expected in files.items():
        relative = PurePosixPath(name)
        path = entry / name
        if (
            relative.is_absolute()
            or '..' in relative.parts
            or name != relative.as_posix()
            or path.is_symlink()
            or not path.resolve().is_relative_to(entry.resolve())
            or not path.is_file()
            or sha256(path) != expected
        ):
            raise DeploymentError('包内源码/脚本摘要不一致：' + name)
    return manifest


# 每次部署独立日志，另写滚动入口摘要；阶段标签用于失败定位而非飞行状态。
class DeploymentLog:
    # 日志名含UTC、PID、动作，独占创建防止覆盖另一部署证据。
    def __init__(self, entry, home, action):
        self.entry = entry
        self.home = home
        self.action = action
        self.stage = 'initialization'
        stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')
        folder = home / 'FC01_navigation' / 'deploy-logs' / entry.name
        folder.mkdir(mode=0o700, parents=True, exist_ok=True)
        self.path = folder / (stamp + '_' + str(os.getpid()) + '_' + action + '.log')
        self.output = self.path.open('x', buffering=1)
        self.summary = home / 'FC01_navigation/install.log'
        self.record('BEGIN ' + action + ' entry=' + str(entry))

    # 持久追加单行阶段消息，UTC用于人类跨记录对齐，不作为控制单调时间。
    def record(self, message):
        self.output.write(datetime.now(timezone.utc).isoformat() + ' ' + message + '\n')

    # 有界保留末18行供错误显示，完整输出写本次日志；Ctrl+C先让子事务安全收尾。
    def step(self, stage, command):
        self.stage = stage
        print('FC01_STEP ' + stage + ' log=' + str(self.path), flush=True)
        self.record(stage + ' command=' + repr(list(map(str, command))))
        recent = deque(maxlen=18)
        with subprocess.Popen(
            list(map(str, command)),
            cwd=self.entry,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            errors='replace',
        ) as child:
            try:
                for line in child.stdout:
                    self.output.write(line)
                    recent.append(line.rstrip())
                code = child.wait()
            except KeyboardInterrupt:
                print('FC01_INTERRUPTED：等待当前步骤安全收尾；不要强制结束控制器。', flush=True)
                child.terminate()
                # The restore process handles SIGTERM through its transaction
                # recovery. Drain its output while it performs a graceful stop.
                for line in child.stdout:
                    self.output.write(line)
                child.wait()
                raise
        self.record(stage + ' exit_status=' + str(code))
        if code:
            for line in recent:
                print(line, file=sys.stderr)
            raise DeploymentError(stage + ' 失败，exit_status=' + str(code))

    # 记录最终结果并关闭本次日志；失败也打印日志路径，便于现场发送原始证据。
    def finish(self, result, error=None):
        message = result + ' stage=' + self.stage + ' log=' + str(self.path)
        if error is not None:
            message += ' error=' + str(error)
        self.record(message)
        with self.summary.open('a') as summary:
            summary.write(datetime.now(timezone.utc).isoformat() + ' ' + message + '\n')
        self.output.close()
        print(message, flush=True)


# 只生成release局部的C++检查输入，不写当前运行配置、不做在线探针或service动作。
def prepare_offline_config(entry):
    """Generate only a release-local checker input; no live config or SDK probe."""
    import yaml
    from navigation_service import _metadata, navigation_profile

    base = yaml.safe_load((entry / 'native/config/flight_test.yaml').read_text())
    selected = json.loads((entry / 'navigation_settings.json').read_text())
    target = entry / 'deployment/flight-test-runtime.yaml'
    config = navigation_profile(base, selected['target_height_m'], selected['navigation_mode_slot'])
    _metadata(
        config,
        'flight-test',
        config_path=target,
        source_path=entry / 'native/config/flight_test.yaml',
    )
    target.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
    data = yaml.safe_dump(config, allow_unicode=True, sort_keys=False)
    if not target.is_file() or target.read_text() != data:
        target.write_text(data)
    return target


# 删除旧成功令牌后重新构建/检查，前后各验源码，最后写与当前清单/二进制/YAML绑定的离线证据。
def offline(entry, home, log):
    release = verify_payload(entry)
    # A failed repeat preparation must not leave a previous success token.
    (entry / 'OFFLINE_DEPLOYMENT.json').unlink(missing_ok=True)
    # This action never imports the service installer/probe or mutates systemd.
    log.step('ARM64_BUILD', ['bash', entry / 'navigation_build_fresh.sh'])
    log.stage = 'OFFLINE_CONFIG_PREPARE'
    config = prepare_offline_config(entry)
    log.step(
        'OFFLINE_CONFIG_CHECK',
        [
            'bash',
            entry / 'navigation_boot.sh',
            'check-config',
            '--mode',
            'flight-test',
            '--confirm-flight-test',
            '--file',
            config,
            '--prefix',
            entry / 'install',
        ],
    )
    verify_payload(entry)
    evidence_path = entry / 'ARM64_BUILD_EVIDENCE.json'
    evidence = json.loads(evidence_path.read_text())
    write_json(
        entry / 'OFFLINE_DEPLOYMENT.json',
        dict(
            schema_version=1,
            release_name=release['release_name'],
            version=release['version'],
            release_manifest_sha256=sha256(entry / 'FC01_RELEASE.json'),
            build_evidence_sha256=sha256(evidence_path),
            controller_sha256=evidence['binaries']['fc01_system_node']['sha256'],
            validation_config_sha256=sha256(config),
            validation_config=str(config),
            validated_at_utc=datetime.now(timezone.utc).isoformat(),
            log=str(log.path),
            controller_executed=False,
            online_probe_executed=False,
            services_modified=False,
            agent_started=False,
            online_activated=False,
        ),
    )
    print('FC01_OFFLINE_READY evidence=' + str(evidence_path), flush=True)
    print('离线部署完成；尚未正式启用。原服务配置和 Agent 未变更。', flush=True)


# activate必须显式确认且离线证据仍与当前清单/配置一致；rollback使用备份事务。
def online(entry, home, log, *, action, confirm=False, boot_policy='preserve', backup=None):
    if action == 'activate':
        verify_payload(entry)
        if not confirm:
            raise DeploymentError(
                '正式启用必须提供 --confirm-flight-test（拆桨、未解锁、接地、RC安全）'
            )
        ready = json.loads((entry / 'OFFLINE_DEPLOYMENT.json').read_text())
        config = entry / 'deployment/flight-test-runtime.yaml'
        if (
            ready.get('schema_version') != 1
            or ready.get('release_manifest_sha256') != sha256(entry / 'FC01_RELEASE.json')
            or ready.get('build_evidence_sha256') != sha256(entry / 'ARM64_BUILD_EVIDENCE.json')
            or ready.get('validation_config') != str(config)
            or not config.is_file()
            or config.is_symlink()
            or ready.get('validation_config_sha256') != sha256(config)
        ):
            raise DeploymentError('离线构建证据已变化，请重新执行 offline；未切换服务')
        command = [
            'bash',
            entry / 'navigation_restore.sh',
            'activate',
            '--confirm-flight-test',
            '--boot-policy',
            boot_policy,
        ]
        log.step('ONLINE_ACTIVATION', command)
    else:
        if backup is None:
            state = json.loads((entry / 'ACTIVATION_STATE.json').read_text())
            backup = Path(state['backup'])
        log.step(
            'SAFE_ROLLBACK',
            ['bash', entry / 'navigation_restore.sh', 'rollback', '--backup', backup],
        )
    state_path = entry / 'ACTIVATION_STATE.json'
    if state_path.is_file():
        state = json.loads(state_path.read_text())
        if state.get('backup'):
            print('FC01_ROLLBACK_PATH=' + str(state['backup']), flush=True)


# 普通用户入口、私有umask、同用户安装锁；在线路径锁由恢复器持有，避免外层提前释放。
def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        'action', choices=('offline', 'activate', 'rollback'), nargs='?', default='offline'
    )
    parser.add_argument('--confirm-flight-test', action='store_true')
    parser.add_argument('--boot-policy', choices=('preserve', 'enable'), default='preserve')
    parser.add_argument('--backup', type=Path)
    args = parser.parse_args(argv)
    if os.getuid() == 0:
        parser.error('请使用 orangepi 普通用户执行；正式启用时脚本会请求 sudo')
    os.umask(0o077)
    home = Path(pwd.getpwuid(os.getuid()).pw_dir)
    log = DeploymentLog(ENTRY, home, args.action)
    print('FC01_DEPLOY_LOG=' + str(log.path), flush=True)
    print('FC01_INSTALL_LOG=' + str(log.summary), flush=True)
    try:
        if args.action == 'offline':
            folder = home / '.cache/fc01_navigation'
            folder.mkdir(mode=0o700, parents=True, exist_ok=True)
            with (folder / 'install.lock').open('a') as lock:
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
                offline(ENTRY, home, log)
        else:
            # navigation_restore owns this same lock for the complete online transaction.
            online(
                ENTRY,
                home,
                log,
                action=args.action,
                confirm=args.confirm_flight_test,
                boot_policy=args.boot_policy,
                backup=args.backup,
            )
    except (
        OSError,
        ValueError,
        KeyError,
        RuntimeError,
        subprocess.SubprocessError,
        KeyboardInterrupt,
    ) as exc:
        state = ENTRY / 'ACTIVATION_STATE.json'
        if state.is_file():
            try:
                print(
                    'FC01_ROLLBACK_PATH=' + str(json.loads(state.read_text()).get('backup', '')),
                    flush=True,
                )
            except (ValueError, OSError):
                pass
        log.finish('FC01_DEPLOY_FAILED', exc)
        return 1
    log.finish('FC01_DEPLOY_COMPLETE')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
