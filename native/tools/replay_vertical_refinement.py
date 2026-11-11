#!/usr/bin/env python3
# 对固定控制日志输入比较旧/新垂直控制输出：高度向上正，原 vz/z_deriv 为 NED 向下正。
# 姿态、控制权和新鲜度是合成的，因此不能归因真实倾斜、接管或预测新飞行高度。
"""Compare production control outputs on fixed controller-log inputs.

Uses recorded height/vz/z_deriv, raw source and detector times at 20 Hz.
Attitude, authority, freshness and admission are synthetic, so this cannot
attribute actual tilt/handovers or predict any new flight trajectory.
Only available current Feedback Guard logs have the required raw timestamps.
"""
import argparse
import csv
import hashlib
import io
import json
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
# CSV 列包含推力构成、限幅和状态，便于比较完整结果而不是只看最终 phase。
FIELDS = ('elapsed_s,phase,publish,thrust,target_height,velocity_target,'
          'closed_loop,takeoff_limited,p_thrust,i_thrust,raw_thrust,thrust_ceiling,reason').split(',')


# 只从 ARMED_GROUND→THRUST_RAMP 后读取连续原始反馈，时间回退/长间隔/缺字段即拒绝。
def read_frames(path):
    history = []
    for line in path.read_text().splitlines():
        try:
            row = json.loads(line)
        except ValueError:
            continue
        if row.get('kind') == 'phase_event':
            history.append(row)
    first = next(i for i, e in enumerate(history)
                 if e['phase_before'] == 'ARMED_GROUND' and e['phase_after'] == 'THRUST_RAMP')
    start = history[first]['monotonic_s']
    frames = []
    previous = start
    for event in history[first+1:]:
        dt = event['monotonic_s'] - previous
        previous = event['monotonic_s']
        if dt <= 0 or dt > .3:
            raise ValueError(f'non-contiguous control frames in {path.name}')
        # dt 单位 s，其余源时间戳保持原整数；保留 detector 与局部位置各自时间，不用新包络刷新旧样本。
        values = [dt, event['height'], event['vz'], event['z_deriv_ned'],
                  event['status_stamp'], event['land_stamp'], event['local_stamp'],
                  event['vertical_feedback_sample_stamp'], int(event['landed']), int(event['at_rest'])]
        if any(v is None for v in values) or not event['vertical_feedback_sample_stamp']:
            raise ValueError(f'missing current raw feedback in {path.name}')
        frames.append((event, values))
        if event['phase_after'] == 'SAFETY':
            break
    if not frames:
        raise ValueError(f'no active flight frames in {path.name}')
    return start, frames


# 给指定控制头文件树编译同一个回放驱动；diagnostics 只控制诊断列，不改变真实输入。
def build(root, path, diagnostics):
    cmd = ['g++', '-std=c++17', '-Wall', '-Wextra', '-Werror', '-pedantic',
           '-I', str(root/'src/jikl_fc01_system/include'), '-I', str(ROOT/'tests')]
    if diagnostics:
        cmd.append('-DREFINEMENT_DIAGNOSTICS')
    subprocess.run(cmd+[str(ROOT/'tests/vertical_control_replay.cpp'), '-o', str(path)], check=True)


# 按既定数值序列驱动离线二进制，并保留原 stdout 与字段名 CSV，不联系飞控。
def run(binary, frames, path):
    stream = ''.join(' '.join(format(v, '.12g') if isinstance(v, float) else str(v)
                             for v in values)+'\n' for _, values in frames)
    result = subprocess.run([str(binary)], input=stream, text=True, capture_output=True, check=True)
    rows = list(csv.DictReader(io.StringIO(','.join(FIELDS)+'\n'+result.stdout)))
    path.write_text(','.join(FIELDS)+'\n'+result.stdout)
    return rows


# 始终运行当前候选；显式提供 baseline_root 时才比较旧树和原日志推力。
def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--controller-log', type=Path, nargs='+', required=True)
    parser.add_argument('--baseline-root', type=Path)
    parser.add_argument('--output-dir', type=Path, required=True)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    summaries = []
    with tempfile.TemporaryDirectory(prefix='fc01_vertical_compare_') as directory:
        candidate = Path(directory)/'candidate'
        baseline = Path(directory)/'baseline'
        build(ROOT, candidate, True)
        if args.baseline_root:
            build(args.baseline_root.resolve(), baseline, False)
        for log in args.controller_log:
            start, frames = read_frames(log)
            new = run(candidate, frames, args.output_dir/(log.stem+'_candidate.csv'))
            old = run(baseline, frames, args.output_dir/(log.stem+'_baseline.csv')) if args.baseline_root else []
            # The real 21:00 terminal is tilt-related. A level-attitude
            # vertical-only comparison intentionally cannot reproduce it.
            comparable = [(e, r) for (e, _), r in zip(frames, old)
                          if e['thrust_planned_z'] is not None and r['publish'] == '1']
            # 只比较原日志有推力且旧程序确实发布的帧；不把缺失输出填零制造假等价。
            error = max((abs(abs(e['thrust_planned_z'])-float(r['thrust']))
                         for e, r in comparable), default=None)
            if error is not None and error > .002:
                raise RuntimeError(f'baseline cannot reproduce recorded vertical output: {log.name}: {error}')
            # 按原 .50 悬停、2 s 斜坡与 .04 储备时序计算超额，检查候选未扩大离地前推力计划。
            def exceed(rows):
                result = []
                for r in rows:
                    t = float(r['elapsed_s'])
                    cap = .50*min(1., t/2.)+.04*max(0., min((t-2.)/2., 1.))
                    if r['phase'] == 'THRUST_RAMP' and r['publish'] == '1':
                        result.append(max(0., float(r['thrust'])-cap))
                return max(result, default=0.)
            entry = {
                'controller_log': log.name, 'sha256': hashlib.sha256(log.read_bytes()).hexdigest(),
                'first_ramp_monotonic_s': start, 'input_frames': len(frames),
                'candidate_frames': len(new), 'baseline_frames': len(old),
                'max_baseline_recorded_thrust_error': error,
                'baseline_max_ramp_schedule_excess': exceed(old) if old else None,
                'candidate_max_ramp_schedule_excess': exceed(new),
                'candidate_takeoff_limited_frames': sum(r['takeoff_limited'] == '1' for r in new),
                'candidate_final_phase': new[-1]['phase'], 'candidate_final_reason': new[-1]['reason'],
                'original_terminal_reason': frames[-1][0]['reason'],
                'interpretation': 'Fixed historical vertical inputs; synthetic level attitude/authority/freshness. '
                                  'No new flight trajectory, actual height or handover attribution.'}
            # 候选越过原推力时序立即失败；摘要的合成限制仍保留，不能将离线通过写成现场验收。
            if entry['candidate_max_ramp_schedule_excess'] > 1e-6:
                raise RuntimeError('candidate exceeded original ramp schedule')
            summaries.append(entry)
    (args.output_dir/'control_replay_summary.json').write_text(
        json.dumps(summaries, ensure_ascii=False, indent=2)+'\n')
    for entry in summaries:
        print(json.dumps(entry, ensure_ascii=False))


if __name__ == '__main__':
    main()
