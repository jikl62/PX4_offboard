#!/usr/bin/env python3
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
BASE_FILES = ('status.json', 'runtime.yaml', 'standby.log', 'standby.log.1', 'standby.log.2',
              'controller.log', 'controller.log.1', 'controller.log.2', 'install.log',
              'install.log.1', 'install.log.2', 'probe.json', 'install-runtime.yaml',
              'velocity_command_diagnostic.json', 'velocity_command_diagnostic.log',
              'velocity_command_diagnostic.log.1')
ENTRY_FILES = ('FC01_RELEASE.json', 'HOST_VERIFICATION.json', 'ARM64_BUILD_EVIDENCE.json',
               'OFFLINE_DEPLOYMENT.json', 'ACTIVATION_STATE.json', 'build-complete.json',
               'deployment-config.json')
MAX_TOTAL = 32 * 1024 * 1024
MAX_LOG = 4 * 1024 * 1024 + 65536
MAX_METADATA = 1024 * 1024


def collect_report(home, *, entry, runner=subprocess.run):
    home, entry = Path(home), Path(entry)
    folder = home/'FC01_navigation'
    folder.mkdir(mode=0o700, exist_ok=True)
    target = home/('FC01_navigation_report_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f')+'.zip')
    started = time.monotonic()
    capture = dict(schema_version=1, started_utc=datetime.now(timezone.utc).isoformat(),
                   invoked_entry=str(entry), non_atomic_live_capture=True, files={}, commands={})
    total = 0
    captured_names = set()
    # Create with private permissions from the first byte, including during
    # compression. Reading logs or evidence never invokes a service action.
    fd = os.open(target, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    with os.fdopen(fd, 'wb') as output, zipfile.ZipFile(output, 'w', zipfile.ZIP_DEFLATED) as archive:
        def include(path, name, *, limit=MAX_METADATA, reserve=0):
            nonlocal total
            if name in captured_names:return None
            captured_names.add(name)
            item = dict(source=str(path))
            capture['files'][name] = item
            if total+reserve >= MAX_TOTAL:
                item['result'] = 'REPORT_SIZE_LIMIT'
                return None
            try:
                read_fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK | os.O_NOFOLLOW)
                with os.fdopen(read_fd, 'rb') as source:
                    before = os.fstat(source.fileno())
                    if not stat.S_ISREG(before.st_mode):
                        item['result'] = 'NOT_REGULAR_FILE'
                        return None
                    cap = min(limit, MAX_TOTAL-total-reserve)
                    offset = max(0, before.st_size-cap)
                    source.seek(offset)
                    data = source.read(cap)
                    after = os.fstat(source.fileno())
                try:
                    current = Path(path).stat()
                    renamed = (current.st_dev, current.st_ino) != (before.st_dev, before.st_ino)
                except OSError:
                    renamed = True
                changed = ((before.st_size, before.st_mtime_ns) != (after.st_size, after.st_mtime_ns)
                           or renamed)
                archive.writestr(name, data)
                total += len(data)
                item.update(result='CAPTURED', bytes=len(data), source_bytes=before.st_size,
                            tail_offset=offset, truncated=bool(offset or len(data)<before.st_size),
                            changed_during_read=changed, mtime_ns=before.st_mtime_ns,
                            sha256=hashlib.sha256(data).hexdigest())
                return data
            except OSError as error:
                item.update(result='UNAVAILABLE', error=str(error))
                return None

        initial = include(folder/'status.json', 'status.json')
        try:
            row = json.loads(initial or b'{}')
            if not isinstance(row, dict):row = {}
        except (ValueError, UnicodeError):
            row = {}
        state = row.get('state') if isinstance(row.get('state'), dict) else {}
        capture['session'] = {key:row.get(key) for key in
                              ('boot_id', 'supervisor_pid', 'controller_pid', 'state_controller_pid',
                               'entry_version', 'runtime_mode', 'feedback_current', 'state_age_seconds')}
        capture['native_state'] = {key:state.get(key) for key in
                                   ('phase', 'reason', 'rc_first_fault_reason', 'rc_latest_fault_reason',
                                    'rc_recovery_block_reason', 'rc_reprepare_block_reason', 'native_log_dropped')}
        try:capture['capture_boot_id'] = Path('/proc/sys/kernel/random/boot_id').read_text().strip()
        except OSError:capture['capture_boot_id'] = None
        # Capture small session/configuration and release evidence before
        # rotating logs can consume the report budget.
        for name in BASE_FILES[1:]:
            if '.log' not in name:include(folder/name, name)

        entries = [('invoked_entry', entry)]
        # A report invoked from a new unpacked release may observe an older
        # running controller. Capture the explicitly named runtime source too.
        source = row.get('config_source_path')
        if isinstance(source, str):
            path = Path(source)
            if path.is_absolute() and len(path.parents)>2 and path.parent.name=='config' and path.parent.parent.name=='native':
                running = path.parents[2]
                if running.is_relative_to(home) and running != entry:
                    entries.append(('running_entry', running))
        capture['entries'] = {label:str(path) for label,path in entries}
        deployment_logs = []
        for label, release in entries:
            for name in ENTRY_FILES:
                include(release/name, label+'/'+name)
            log_folder = folder/'deploy-logs'/release.name
            try:
                logs = sorted((p for p in log_folder.glob('*.log') if not p.is_symlink()),
                              key=lambda p:p.stat().st_mtime_ns, reverse=True)[:6]
            except OSError:
                logs = []
            for path in logs:
                deployment_logs.append((path, 'deploy-logs/'+release.name+'/'+path.name))

        for name in BASE_FILES:
            if '.log' in name:include(folder/name, name, limit=MAX_LOG, reserve=MAX_METADATA)
        for path, name in deployment_logs:
            include(path, name, limit=MAX_METADATA, reserve=MAX_METADATA)

        final = include(folder/'status.json', 'status_after_capture.json')
        capture['status_changed_during_capture'] = initial != final if initial is not None and final is not None else None
        for name, argv in (
                ('service.txt', ['/usr/bin/systemctl', 'status', SERVICE, '--no-pager']),
                ('journal.txt', ['/usr/bin/journalctl', '-u', SERVICE, '-n', '100', '--no-pager'])):
            try:
                result = runner(argv, capture_output=True, text=True, timeout=10)
                data = (result.stdout+result.stderr).encode('utf8', errors='replace')[-262144:]
                capture['commands'][name] = dict(argv=argv, returncode=result.returncode)
            except (OSError, subprocess.SubprocessError) as error:
                data = str(error).encode('utf8', errors='replace')
                capture['commands'][name] = dict(argv=argv, error=str(error))
            archive.writestr(name, data)
        capture.update(finished_utc=datetime.now(timezone.utc).isoformat(),
                       elapsed_s=max(0, time.monotonic()-started), captured_file_bytes=total)
        archive.writestr('REPORT_CAPTURE.json', json.dumps(capture, ensure_ascii=False, indent=2)+'\n')
    print('自动报告：'+str(target))
    return target
