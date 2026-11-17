#!/usr/bin/env python3
"""列出本机已有 ULog，按内容去重，只做模式初筛；不访问硬件或网络。"""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
from pyulog import ULog


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    cmd = ['rg', '--files', '--hidden', '/home/jikl62', '-g', '*.ulg', '-g', '!.git',
           '-g', '!node_modules']
    search = subprocess.run(cmd, text=True, capture_output=True, check=True)
    paths = sorted(Path(p) for p in search.stdout.splitlines())
    excluded = [p for p in paths if '/.local/share/Trash/' in str(p) or '/docs/assets/' in str(p)]
    selected = [p for p in paths if p not in excluded]
    unique = {}
    for path in selected:
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
        entry = unique.setdefault(digest, {'sha256': digest, 'size': path.stat().st_size, 'paths': []})
        entry['paths'].append(str(path))
    inventory = {'all_paths': len(paths), 'excluded_trash_or_px4_sample': len(excluded),
                 'selected_paths': len(selected), 'unique': len(unique),
                 'files': list(unique.values()), 'discovery_command': cmd,
                 'discovery_stderr': search.stderr, 'excluded_paths': [str(p) for p in excluded],
                 'limits': 'No external mounts or compressed-archive traversal; filename alone does not establish hardware identity.'}
    classified = []
    for entry in inventory['files']:
        path = entry['paths'][0]
        try:
            log = ULog(path, message_name_filter_list=['vehicle_status', 'vehicle_status_v1'])
            modes, intervals = set(), []
            for topic in log.data_list:
                if topic.name not in ('vehicle_status', 'vehicle_status_v1'):
                    continue
                nav, stamps = topic.data['nav_state'], topic.data['timestamp']
                modes.update(int(v) for v in nav)
                for i, value in enumerate(nav):
                    if int(value) != 2 or (i > 0 and int(nav[i-1]) == 2):
                        continue
                    end = i + 1
                    while end < len(nav) and int(nav[end]) == 2:
                        end += 1
                    intervals.append({'start_us': int(stamps[i]),
                                      'end_us': int(stamps[end]) if end < len(nav) else None,
                                      'samples': end-i})
            classified.append({'source': path, 'sha256': entry['sha256'], 'nav_states': sorted(modes),
                               'position_intervals': intervals, 'firmware': log.msg_info_dict.get('ver_sw'),
                               'hardware': log.msg_info_dict.get('ver_hw'),
                               'is_sitl_label': 'SITL' in path})
        except Exception as error:
            classified.append({'source': path, 'error': repr(error)})
    for name, data in [('ULOG_INVENTORY.json', inventory), ('ULOG_EXTRA_CLASSIFICATION.json', classified)]:
        (args.output/name).write_text(json.dumps(data, ensure_ascii=False, indent=2) + '\n')
    print('unique=', len(classified), 'Position-bearing=',
          sum(bool(r.get('position_intervals')) for r in classified))


if __name__ == '__main__':
    main()
