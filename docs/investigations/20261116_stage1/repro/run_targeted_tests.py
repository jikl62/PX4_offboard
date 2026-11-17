#!/usr/bin/env python3
"""离线编译并运行本阶段相关测试；输出只写指定证据目录和临时二进制。"""
import argparse
import concurrent.futures
import json
from pathlib import Path
import subprocess
import tempfile
import time

BASE = Path(__file__).resolve().parents[1]
REPO = BASE.parents[2]
NAMES = [
    'test_takeoff_evidence', 'test_r632_ramp_height_guard', 'test_r632_height_tracking',
    'test_vertical_feedback_guard', 'test_rc_navigation', 'test_rc_takeoff_stage',
    'test_r62c_review_clock', 'test_stage3_publication_combinations',
]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='fc01-stage1-tests-') as build:
        def run(name):
            started = time.monotonic()
            exe = str(Path(build) / name)
            cmd = ['g++', '-std=c++17', '-O2', '-Wall', '-Wextra', '-I',
                   'native/src/jikl_fc01_system/include', '-I', 'native/tests',
                   'native/tests/' + name + '.cpp', '-o', exe]
            compile_result = subprocess.run(cmd, cwd=REPO, capture_output=True, text=True)
            result = {'name': name, 'build_command': cmd, 'build_exit': compile_result.returncode}
            (args.output / (name + '.build.log')).write_text(
                compile_result.stdout + compile_result.stderr)
            if compile_result.returncode == 0:
                test = subprocess.run([exe], cwd=REPO, capture_output=True, text=True, timeout=60)
                result.update(test_exit=test.returncode, stdout=test.stdout, stderr=test.stderr)
                (args.output / (name + '.log')).write_text(test.stdout + test.stderr)
            result['elapsed_seconds'] = round(time.monotonic() - started, 3)
            return result
        with concurrent.futures.ThreadPoolExecutor(max_workers=3) as pool:
            results = list(pool.map(run, NAMES))
    (args.output / 'results.json').write_text(json.dumps(results, indent=2) + '\n')
    for result in results:
        print(result['name'], result['build_exit'], result.get('test_exit'))
    return int(any(r['build_exit'] or r.get('test_exit', 1) for r in results))


if __name__ == '__main__':
    raise SystemExit(main())
