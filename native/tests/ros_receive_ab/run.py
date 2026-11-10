#!/usr/bin/env python3
"""Opt-in real Humble executor A/B; localhost-only Domain 188, no /fmu or UART."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import tempfile
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--build-dir', type=Path, required=True)
    parser.add_argument('--input-hz', type=float, default=25.)
    parser.add_argument('--seconds', type=float, default=6.)
    parser.add_argument('--output', type=Path, help='write JSON report; CSV uses an adjacent folder')
    args = parser.parse_args()
    if not 20 < args.input_hz <= 40 or not 4 <= args.seconds <= 30:
        parser.error('input-hz must be >20..40 and seconds must be 4..30')
    binary = (args.build_dir/'fc01_receive_ab').resolve()
    if not binary.is_file():
        parser.error('build the standalone CMake target first')
    env = os.environ.copy()
    env.update(ROS_DOMAIN_ID='188', ROS_LOCALHOST_ONLY='1', RMW_IMPLEMENTATION='rmw_fastrtps_cpp')
    for name in ('ROS_DISCOVERY_SERVER', 'CYCLONEDDS_URI', 'FASTRTPS_DEFAULT_PROFILES_FILE',
                 'FASTDDS_DEFAULT_PROFILES_FILE', 'RMW_FASTRTPS_USE_QOS_FROM_XML', 'ROS_STATIC_PEERS'):
        env.pop(name, None)
    with tempfile.TemporaryDirectory(prefix='fc01-receive-ab-') as temporary:
        csv_dir = args.output.with_suffix('.csv') if args.output else Path(temporary)
        csv_dir.mkdir(parents=True, exist_ok=True)
        results = []

        def run(mode, lag=0, callback=0, seconds=None):
            seconds = args.seconds if seconds is None else seconds
            command = [str(binary), mode, str(args.input_hz), str(seconds), '5',
                       str(csv_dir/f'{mode}-{lag}-{callback}.csv'), str(lag), str(callback)]
            sub = subprocess.Popen(command, env=env, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            pub = None
            try:
                time.sleep(.2)
                pub = subprocess.Popen([str(binary), 'pub', str(args.input_hz), str(seconds+1), '5',
                                        '/dev/null', str(lag)], env=env, text=True,
                                       stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                stdout, stderr = sub.communicate(timeout=seconds+8)
                if sub.returncode:
                    raise RuntimeError(f'{mode}: {sub.returncode}: {stderr}\n{stdout}')
                result = json.loads(stdout.strip().splitlines()[-1])
                result.update(raw_lag_ms=lag, callback_work_ms=callback)
                results.append(result)
                print(json.dumps(result), flush=True)
                return result
            finally:
                for process in (pub, sub):
                    if process is not None:
                        if process.poll() is None:
                            process.terminate()
                        process.communicate(timeout=3)

        old = run('some')
        persistent = run('persistent_some')
        all_work = run('all')
        repeated = run('repeated')
        stale = run('repeated', lag=600, seconds=2)
        slow = run('repeated', callback=12, seconds=2)
        empty_process = subprocess.run([str(binary), 'empty', '0', '0', '5', str(csv_dir/'empty.csv')],
                                       env=env, text=True, capture_output=True, timeout=6)
        if empty_process.returncode:
            raise RuntimeError(empty_process.stderr)
        empty = json.loads(empty_process.stdout.strip())
        results.append(empty)
        checks = dict(
            one_sample_per_subscription_per_some_pass=old['max_callbacks_per_cycle'] == persistent['max_callbacks_per_cycle'] == 1,
            spin_all_drains=all_work['max_callbacks_per_cycle'] >= 2,
            repeated_drains=repeated['max_callbacks_per_cycle'] >= 2,
            repeated_does_not_accumulate=repeated['callback_age_last_ms'] < 100,
            old_accumulates=old['callback_age_last_ms'] > 100,
            original_sample_stays_invalid=stale['envelope_valid_count'] == stale['count'] and stale['raw_sample_valid_count'] == 0,
            cooperative_budget_can_overrun=slow['spin_max_ms'] > 5 and slow['budget_hits'] > 0,
            no_data_does_not_busy_wait=empty['spin_mean_ms'] < 5.)
        report = dict(isolation='localhost Domain 188; ordinary synthetic topic only; no control, Agent or serial',
                      input_hz=args.input_hz, results=results, checks=checks, passed=all(checks.values()))
        if args.output:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(json.dumps(report, indent=2)+'\n')
        print(json.dumps({'passed': report['passed'], 'checks': checks}), flush=True)
        return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
