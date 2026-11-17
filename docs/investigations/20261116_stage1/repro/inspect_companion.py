#!/usr/bin/env python3
"""只读解析已有香橙派报告；不启动服务、不修改配置、不连接设备。"""
import collections
import hashlib
import json
from pathlib import Path
import zipfile

BASE = Path(__file__).resolve().parents[1]
REPO = BASE.parents[2]
ARCHIVES = [
    Path('/home/jikl62/桌面/FC01_navigation_report_20261114_093631_680121.zip'),
    Path('/home/jikl62/桌面/FC01_navigation_report_20261021_152318_747738.zip'),
]
FIELDS = (
    'phase', 'reason', 'controller_pid', 'diagnostic_monotonic_s', 'diagnostic_host_us',
    'nav_state', 'armed', 'landed', 'at_rest', 'rc_mode_slot', 'rc_arm_switch',
    'height_reference_z0', 'height_sample_z', 'relative_height_m', 'velocity_ned',
    'z_deriv_ned', 'height_reference_quat_reset', 'attitude_reset_counter',
    'height_sample_z_reset', 'height_sample_vz_reset', 'local_timestamp_sample',
    'liftoff_confirmed', 'target_height_settled', 'liftoff_unconfirmed_reason',
    'heartbeat_enabled', 'attitude_output_enabled', 'publication_block_reason',
    'last_heartbeat_local_commit_monotonic_s', 'last_target_local_commit_monotonic_s',
    'rc_first_fault_reason', 'rc_latest_fault_reason', 'source_clock_valid',
    'source_clock_validated', 'vertical_feedback_reason', 'source_timing',
)


def inspect(path):
    result = {'source': str(path), 'sha256': hashlib.sha256(path.read_bytes()).hexdigest()}
    with zipfile.ZipFile(path) as archive:
        result['zip_crc_ok'] = archive.testzip() is None
        release = json.loads(archive.read('invoked_entry/FC01_RELEASE.json'))
        result['release'] = release
        result['local_source_sha256_comparison'] = {
            name: {
                'expected': digest,
                'actual': hashlib.sha256((REPO / name).read_bytes()).hexdigest()
                if (REPO / name).is_file() else None,
            }
            for name, digest in release['source_sha256'].items()
        }
        result['source_files_match'] = all(
            item['expected'] == item['actual']
            for item in result['local_source_sha256_comparison'].values()
        )
        result['service'] = archive.read('service.txt').decode()
        result['journal'] = archive.read('journal.txt').decode()
        result['runtime_yaml'] = archive.read('runtime.yaml').decode()
        status = json.loads(archive.read('status.json'))
        result['status'] = {k: v for k, v in status.items() if k != 'state'}
        result['state'] = {k: status.get('state', {}).get(k) for k in FIELDS}
        for name in ('ARM64_BUILD_EVIDENCE.json', 'OFFLINE_DEPLOYMENT.json'):
            member = 'invoked_entry/' + name
            if member in archive.namelist():
                data = json.loads(archive.read(member))
                result[name] = {k: data[k] for k in (
                    'source_commit', 'host_architecture', 'built_at_utc', 'sdk_root',
                    'input_sha256', 'controller_sha256', 'binaries', 'controller_executed',
                    'agent_or_service_started', 'online_activated',
                ) if k in data}
        logs = []
        for name in ('controller.log.2', 'controller.log.1', 'controller.log'):
            if name not in archive.namelist():
                continue
            rows, transitions, events, counts = [], [], [], collections.Counter()
            previous = None
            for number, line in enumerate(archive.read(name).decode().splitlines(), 1):
                # 日志前有 Python 墙钟前缀；它不是 JSON 的一部分。
                if '{' not in line:
                    continue
                prefix, payload = line.split('{', 1)
                try:
                    row = json.loads('{' + payload)
                except json.JSONDecodeError:
                    continue
                if row.get('kind') == 'phase_event':
                    events.append(dict(row, line=number, wall_prefix=prefix))
                # 周期 state 与 IPC reply 都带快照；phase_event 另存，不能重复计成传感器样本。
                if 'phase' not in row:
                    continue
                selected = {k: row.get(k) for k in FIELDS}
                selected.update(line=number, wall_prefix=prefix)
                rows.append(selected)
                counts[row.get('phase')] += 1
                key = (row.get('phase'), row.get('reason'))
                if key != previous:
                    transitions.append(selected)
                    previous = key
            logs.append({'name': name, 'state_rows': len(rows), 'phase_counts': dict(counts),
                         'first': rows[0] if rows else None, 'last': rows[-1] if rows else None,
                         'transitions': transitions,
                         'phase_events': events,
                         'ascend_rows': [r for r in rows if r['phase'] == 'ASCEND']})
        result['logs'] = logs
    return result


if __name__ == '__main__':
    results = [inspect(path) for path in ARCHIVES]
    output = BASE / 'evidence/COMPANION_RECHECK.json'
    output.write_text(json.dumps(results, ensure_ascii=False, indent=2, allow_nan=False) + '\n')
    for result in results:
        print(Path(result['source']).name, result['release']['version'],
              'source_match=', result['source_files_match'],
              [(log['name'], log['state_rows'], log['phase_counts']) for log in result['logs']])
