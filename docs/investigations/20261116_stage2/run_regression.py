#!/usr/bin/env python3
"""阶段3可直接复跑的离线回归；只构建临时程序，不启动ROS节点或访问设备。"""
import argparse
import concurrent.futures
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time

REPO = Path(__file__).resolve().parents[3]
BASELINE = '3677b9dd1f35e092e43501d26164c6263545c947'
NAMES = [
    'test_20261116_takeoff_expiry', 'test_20261116_diagnostics', 'test_20261116_height_contract',
    'test_takeoff_evidence', 'test_r632_ramp_height_guard', 'test_r632_height_tracking',
    'test_vertical_feedback_guard', 'test_rc_navigation', 'test_rc_takeoff_stage',
    'test_r62c_review_clock', 'test_stage3_publication_combinations',
    'test_stage3_state_invariants', 'test_attitude_vertical',
]


def identity():
    paths = sorted(p for root in ('native/src', 'native/tests', 'native/config')
                   for p in (REPO / root).rglob('*')
                   if p.is_file() and '__pycache__' not in p.parts)
    paths += [REPO / 'tests/test_20261116_stage2_equivalence.py', Path(__file__)]
    hashes = {str(p.relative_to(REPO)): hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
    return {'head': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=REPO, text=True).strip(),
            'files': hashes}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    before = identity()
    results = []

    def command(label, cmd, cwd=REPO, expected=0, env=None):
        started = time.monotonic()
        run = subprocess.run(cmd, cwd=cwd, env=env, text=True, capture_output=True, timeout=240)
        (output / (label + '.log')).write_text(run.stdout + run.stderr)
        return {'name': label, 'command': cmd, 'cwd': str(cwd), 'exit': run.returncode,
                'expected_exit': expected, 'passed': run.returncode == expected,
                'elapsed_s': round(time.monotonic() - started, 3)}

    with tempfile.TemporaryDirectory(prefix='fc01-stage2-check-') as tmp:
        build = Path(tmp)

        def cpp(name, flags=None, include=None, source=None, label=None):
            label = label or name
            exe = str(build / label)
            cmd = ['g++', '-std=c++17', '-Wall', '-Wextra', '-Werror', '-pedantic']
            cmd += flags or ['-O2']
            if include:
                cmd += ['-I', str(include)]
            cmd += ['-I', 'native/src/jikl_fc01_system/include', '-I', 'native/tests',
                    str(source or ('native/tests/' + name + '.cpp')), '-o', exe]
            compiled = command(label + '.build', cmd)
            if not compiled['passed']:
                return [compiled]
            return [compiled, command(label, [exe])]

        with concurrent.futures.ThreadPoolExecutor(max_workers=3) as pool:
            for group in pool.map(cpp, NAMES):
                results.extend(group)
        # 同一个缺陷复现源码分别链接基线/当前生产头；基线退出1为预期失败证据。
        old = build / 'baseline'
        old.mkdir()
        with (build / 'baseline.tar').open('wb') as archive_file:
            subprocess.run(['git', 'archive', BASELINE, 'native'], cwd=REPO,
                           stdout=archive_file, check=True)
        import tarfile
        with tarfile.open(build / 'baseline.tar') as archive:
            archive.extractall(old)
        repro = REPO / 'docs/investigations/20261116_stage2/repro/expired_rise.cpp'
        for label, include, expected in (
            ('repro-baseline', old / 'native/src/jikl_fc01_system/include', 1),
            ('repro-current', REPO / 'native/src/jikl_fc01_system/include', 0),
        ):
            # 使用相同当前测试夹具，只有生产include切换；编译失败绝不是预期复现。
            compiled = command(label + '.build', ['g++', '-std=c++17', '-O2', '-Wall', '-Wextra',
                '-Werror', '-pedantic', '-I', str(include), '-I', 'native/tests', str(repro),
                '-o', str(build / label)])
            results.append(compiled)
            if compiled['passed']:
                results.append(command(label, [str(build / label)], expected=expected))
        for name in ('test_20261116_takeoff_expiry', 'test_20261116_diagnostics',
                     'test_rc_navigation', 'test_stage3_state_invariants'):
            results.extend(cpp(name, flags=['-O1', '-g', '-fno-omit-frame-pointer', '-no-pie',
                '-fsanitize=address,undefined', '-fno-sanitize-recover=all'], label=name + '.san'))
        results.append(command('receive-event-adapter', [sys.executable, '-m', 'unittest',
            'test_receive_event_adapter', '-v'], cwd=REPO / 'native/tests'))
        env = os.environ.copy()
        env['FC01_20261116_STAGE2_TRACE_DIR'] = str(output / 'differential')
        results.append(command('differential', [sys.executable, '-m', 'unittest', 'discover',
            '-s', 'tests', '-p', 'test_20261116_stage2_equivalence.py', '-v'], env=env))
    after = identity()
    record = {'baseline': BASELINE, 'before': before, 'after': after,
              'source_unchanged_during_run': before == after, 'results': results,
              'passed': before == after and all(r['passed'] for r in results),
              'hardware_dds_or_flight_test': False}
    (output / 'results.json').write_text(json.dumps(record, ensure_ascii=False, indent=2) + '\n')
    for r in results:
        print(r['name'], 'PASS' if r['passed'] else 'FAIL', r['exit'])
    print('source_unchanged_during_run', before == after)
    return 0 if record['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
