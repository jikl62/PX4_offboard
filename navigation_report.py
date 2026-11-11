#!/usr/bin/env python3
# 有界只读导航报告：同时保存运行快照、配置、发布/构建身份、部署日志和服务只读查询。
# 活体抓取不是原子快照，报告标记读取期间变化与截断；采集不停止服务、不连接控制socket。
"""Bounded, read-only capture of navigation state and deployment provenance."""
from __future__ import annotations

from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import stat
import subprocess
import time
import zipfile

SERVICE = 'fc01-navigation.service'
# 优先保留小型身份/配置，再收有大小预算的日志；视觉诊断材料与飞控采用证据分开。
BASE_FILES = (
    'status.json',
    'runtime.yaml',
    'standby.log',
    'standby.log.1',
    'standby.log.2',
    'controller.log',
    'controller.log.1',
    'controller.log.2',
    'install.log',
    'install.log.1',
    'install.log.2',
    'probe.json',
    'install-runtime.yaml',
    'velocity_command_diagnostic.json',
    'velocity_command_diagnostic.log',
    'velocity_command_diagnostic.log.1',
)
ENTRY_FILES = (
    'FC01_RELEASE.json',
    'HOST_VERIFICATION.json',
    'ARM64_BUILD_EVIDENCE.json',
    'OFFLINE_DEPLOYMENT.json',
    'ACTIVATION_STATE.json',
    'build-complete.json',
    'deployment-config.json',
)
# 总文件预算32MiB，单日志约4MiB，元数据1MiB；裁尾仍记录原大小和偏移。
MAX_TOTAL = 32 * 1024 * 1024
MAX_LOG = 4 * 1024 * 1024 + 65536
MAX_METADATA = 1024 * 1024


class _ReportFileCapture:
    """单份报告的文件预算与去重所有者；不查询或改变服务，不跨报告共享状态。"""

    def __init__(self, capture):
        """绑定可变采集索引，初始化字节预算；archive在私有ZIP打开后由调用者设置。"""
        self.capture = capture
        self.total = 0
        self.captured_names = set()
        self.archive = None

    # 统一文件入口：名字去重、预算预留、NOFOLLOW、普通文件检查和SHA，失败也记录原因。
    def include(self, path, name, *, limit=MAX_METADATA, reserve=0):
        """捕获一份普通文件并返回字节或None；记录去重、预算与变更证据。

        先标记名字再尝试读取，NOFOLLOW及裁尾/预留顺序保留；只修改本收集器和capture索引。
        """
        if name in self.captured_names:
            return None
        self.captured_names.add(name)
        item = dict(source=str(path))
        self.capture['files'][name] = item
        if self.total + reserve >= MAX_TOTAL:
            item['result'] = 'REPORT_SIZE_LIMIT'
            return None
        try:
            read_fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK | os.O_NOFOLLOW)
            with os.fdopen(read_fd, 'rb') as source:
                before = os.fstat(source.fileno())
                if not stat.S_ISREG(before.st_mode):
                    item['result'] = 'NOT_REGULAR_FILE'
                    return None
                cap = min(limit, MAX_TOTAL - self.total - reserve)
                # 日志太大读取尾部，保留最近故障；不能把截取的摘要当作完整原文件SHA。
                offset = max(0, before.st_size - cap)
                source.seek(offset)
                data = source.read(cap)
                after = os.fstat(source.fileno())
            try:
                current = Path(path).stat()
                renamed = (current.st_dev, current.st_ino) != (before.st_dev, before.st_ino)
            except OSError:
                renamed = True
            # 通过读前/后大小mtime与当前路径inode标记轮转或并发写，避免把混合时段假装原子证据。
            changed = (before.st_size, before.st_mtime_ns) != (
                after.st_size,
                after.st_mtime_ns,
            ) or renamed
            self.archive.writestr(name, data)
            self.total += len(data)
            item.update(
                result='CAPTURED',
                bytes=len(data),
                source_bytes=before.st_size,
                tail_offset=offset,
                truncated=bool(offset or len(data) < before.st_size),
                changed_during_read=changed,
                mtime_ns=before.st_mtime_ns,
                sha256=hashlib.sha256(data).hexdigest(),
            )
            return data
        except OSError as error:
            item.update(result='UNAVAILABLE', error=str(error))
            return None


def _report_entries(home, entry, row):
    """返回调用入口和受控home下的运行版本入口；row只读。

    只识别既有config/native来源路径，不把新包名冒充已部署版本，也不改变服务。
    """
    entries = [('invoked_entry', entry)]
    # A report invoked from a new unpacked release may observe an older
    # running controller. Capture the explicitly named runtime source too.
    source = row.get('config_source_path')
    if isinstance(source, str):
        path = Path(source)
        if (
            path.is_absolute()
            and len(path.parents) > 2
            and path.parent.name == 'config'
            and path.parent.parent.name == 'native'
        ):
            running = path.parents[2]
            if running.is_relative_to(home) and running != entry:
                entries.append(('running_entry', running))
    return entries


def _include_command_report(archive, capture, name, argv, runner):
    """按原10秒超时执行一项只读查询，裁尾写入ZIP并记录返回码/异常。runner可注入；不重试、不发启停命令，调用顺序由收集器保持。"""
    try:
        result = runner(argv, capture_output=True, text=True, timeout=10)
        data = (result.stdout + result.stderr).encode('utf8', errors='replace')[-262144:]
        capture['commands'][name] = dict(argv=argv, returncode=result.returncode)
    except (OSError, subprocess.SubprocessError) as error:
        data = str(error).encode('utf8', errors='replace')
        capture['commands'][name] = dict(argv=argv, error=str(error))
    archive.writestr(name, data)


# 从home/entry读取证据到新建0600 ZIP，返回路径；runner可注入离线fixture且只执行只读查询。
def collect_report(home, *, entry, runner=subprocess.run):
    home, entry = Path(home), Path(entry)
    folder = home / 'FC01_navigation'
    folder.mkdir(mode=0o700, exist_ok=True)
    target = home / (
        'FC01_navigation_report_' + datetime.now().strftime('%Y%m%d_%H%M%S_%f') + '.zip'
    )
    started = time.monotonic()
    capture = dict(
        schema_version=1,
        started_utc=datetime.now(timezone.utc).isoformat(),
        invoked_entry=str(entry),
        non_atomic_live_capture=True,
        files={},
        commands={},
    )
    files = _ReportFileCapture(capture)
    # Create with private permissions from the first byte, including during
    # compression. Reading logs or evidence never invokes a service action.
    fd = os.open(target, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    with (
        os.fdopen(fd, 'wb') as output,
        zipfile.ZipFile(output, 'w', zipfile.ZIP_DEFLATED) as archive,
    ):
        files.archive = archive
        include = files.include
        initial = include(folder / 'status.json', 'status.json')
        try:
            row = json.loads(initial or b'{}')
            if not isinstance(row, dict):
                row = {}
        except (ValueError, UnicodeError):
            row = {}
        state = row.get('state') if isinstance(row.get('state'), dict) else {}
        capture['session'] = {
            key: row.get(key)
            for key in (
                'boot_id',
                'supervisor_pid',
                'controller_pid',
                'state_controller_pid',
                'entry_version',
                'runtime_mode',
                'feedback_current',
                'state_age_seconds',
            )
        }
        capture['native_state'] = {
            key: state.get(key)
            for key in (
                'phase',
                'reason',
                'rc_first_fault_reason',
                'rc_latest_fault_reason',
                'rc_recovery_block_reason',
                'rc_reprepare_block_reason',
                'native_log_dropped',
            )
        }
        try:
            capture['capture_boot_id'] = Path('/proc/sys/kernel/random/boot_id').read_text().strip()
        except OSError:
            capture['capture_boot_id'] = None
        # Capture small session/configuration and release evidence before
        # rotating logs can consume the report budget.
        for name in BASE_FILES[1:]:
            if '.log' not in name:
                include(folder / name, name)

        # 调用入口可能与正在运行的旧版本不同；两者身份分别保留，不用新包名冒充部署成功。
        entries = _report_entries(home, entry, row)
        capture['entries'] = {label: str(path) for label, path in entries}
        deployment_logs = []
        for label, release in entries:
            for name in ENTRY_FILES:
                include(release / name, label + '/' + name)
            log_folder = folder / 'deploy-logs' / release.name
            try:
                logs = sorted(
                    (p for p in log_folder.glob('*.log') if not p.is_symlink()),
                    key=lambda p: p.stat().st_mtime_ns,
                    reverse=True,
                )[:6]
            except OSError:
                logs = []
            for path in logs:
                deployment_logs.append((path, 'deploy-logs/' + release.name + '/' + path.name))

        for name in BASE_FILES:
            if '.log' in name:
                include(folder / name, name, limit=MAX_LOG, reserve=MAX_METADATA)
        for path, name in deployment_logs:
            include(path, name, limit=MAX_METADATA, reserve=MAX_METADATA)

        final = include(folder / 'status.json', 'status_after_capture.json')
        capture['status_changed_during_capture'] = (
            initial != final if initial is not None and final is not None else None
        )
        # systemctl status/journalctl只是读取，各自超时10s并裁尾；不会重启或更改systemd。
        for name, argv in (
            ('service.txt', ['/usr/bin/systemctl', 'status', SERVICE, '--no-pager']),
            ('journal.txt', ['/usr/bin/journalctl', '-u', SERVICE, '-n', '100', '--no-pager']),
        ):
            _include_command_report(archive, capture, name, argv, runner)
        capture.update(
            finished_utc=datetime.now(timezone.utc).isoformat(),
            elapsed_s=max(0, time.monotonic() - started),
            captured_file_bytes=files.total,
        )
        # 最后写采集索引，含原来源、结果、字节数、截断/变化和捕获时间；不能替代同程PX4 ULog。
        archive.writestr(
            'REPORT_CAPTURE.json', json.dumps(capture, ensure_ascii=False, indent=2) + '\n'
        )
    print('自动报告：' + str(target))
    return target
