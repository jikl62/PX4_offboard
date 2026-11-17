#!/usr/bin/env python3
"""对已导出 ULog 样本重放生产守卫。日志采样不能代替实际 DDS 输入序列。"""
import csv
import json
from pathlib import Path
import subprocess

BASE = Path(__file__).resolve().parents[1]
EXECUTABLE = '/tmp/fc01-stage1-vertical-feedback-replay'
results = []
for source in sorted((BASE / 'evidence/ulog').glob('*_local_position.csv')):
    rows = list(csv.DictReader(source.open()))
    # Core 在 active reset 时先停流；这里按垂向 reset 分段，只观察各连续参考内的守卫。
    segments = []
    for row in rows:
        reset = (row['z_reset_counter'], row['vz_reset_counter'])
        if not segments or segments[-1]['reset'] != reset:
            segments.append({'reset': reset, 'rows': []})
        segments[-1]['rows'].append(row)
    output = []
    for number, segment in enumerate(segments):
        data = ''.join(
            f"{r['timestamp_sample']} {r['z']} {r['vz']} {r['z_deriv']}\n"
            for r in segment['rows']
        )
        run = subprocess.run([EXECUTABLE], input=data, text=True, capture_output=True, check=True)
        lines = run.stdout.splitlines()
        last = lines[-1].split(',') if lines else []
        output.append({'reset': segment['reset'], 'input_rows': len(segment['rows']),
                       'consumed_rows': len(lines), 'last_result': last,
                       'healthy_at_end': bool(last) and last[1] == '1',
                       'rate_warning_rows': sum(line.split(',')[2] == '1' for line in lines)})
        target = BASE / 'evidence/vertical_replay' / f'{source.stem}_segment{number}.csv'
        target.parent.mkdir(exist_ok=True)
        target.write_text('sample_us,healthy,suspect,rate_error_mps,position_rate_error_mps,disagreement_s,reason\n' + run.stdout)
    results.append({'input': str(source.relative_to(BASE)), 'segments': output})
(BASE / 'evidence/vertical_replay/results.json').write_text(
    json.dumps({'scope': 'Logged and potentially downsampled ULog samples, not full DDS or physical height proof',
                'results': results}, ensure_ascii=False, indent=2) + '\n')
for item in results:
    print(Path(item['input']).name, [(s['consumed_rows'], s['healthy_at_end'], s['last_result'][-1])
                                    for s in item['segments']])
