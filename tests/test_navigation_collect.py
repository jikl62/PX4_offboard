import hashlib
import importlib
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class NavigationCollectTests(unittest.TestCase):
    def api(self):
        self.assertTrue((ROOT/'navigation_collect.py').is_file(), 'passive collector is missing')
        return importlib.import_module('navigation_collect')

    def test_help_works_without_ros(self):
        self.api()
        result = subprocess.run([sys.executable, str(ROOT/'navigation_collect.py'), '--help'],
                                capture_output=True, text=True, timeout=5,
                                env={'PATH': '/nonexistent', 'HOME': '/nonexistent'})
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('--ros-seconds', result.stdout)

    def test_missing_tools_and_timeout_are_unknown(self):
        m = self.api()
        missing = m.run_command(['/definitely/missing/fc01-tool'], timeout=.1)
        self.assertEqual(missing['status'], 'unknown')
        stalled = m.run_command([sys.executable, '-c', 'import time; time.sleep(10)'], timeout=.05)
        self.assertEqual(stalled['status'], 'unknown')
        self.assertIn('timeout', stalled['reason'])

    def test_sample_age_and_cdr_rate_do_not_claim_uart_throughput(self):
        m = self.api(); stats = m.TopicStats()
        stats.observe(1., 100., 99_900_000, 99_800_000, 49)
        stats.observe(1.1, 100.1, 100_000_000, 99_800_000, 51)
        row = stats.report(2.)
        self.assertEqual(row['messages'], 2)
        self.assertAlmostEqual(row['receive_hz'], 1.)
        self.assertAlmostEqual(row['receive_gap_s']['max'], .1)
        self.assertAlmostEqual(row['sample_age_s']['max'], .3)
        self.assertEqual(row['sample_stamp_repeats'], 1)
        self.assertEqual(row['cdr_bytes_per_s'], 50.)
        self.assertNotIn('uart_bytes_per_s', row)

    def test_absent_and_future_timestamps_remain_distinguishable(self):
        m = self.api(); stats = m.TopicStats()
        stats.observe(1., 100., 0, None, 20)
        stats.observe(2., 100., 101_000_000, 101_000_000, 20)
        row = stats.report(2.)
        self.assertEqual(row['message_age_s']['count'], 1)
        self.assertEqual(row['sample_age_s']['min'], -1.)
        self.assertIsNone(m.TopicStats().report(1.)['sample_age_s']['max'])

    def test_service_identity_and_running_binary_are_collected_without_mutation(self):
        m = self.api()
        with tempfile.TemporaryDirectory() as d:
            base = Path(d); entry = base/'active entry'; entry.mkdir()
            project = base/'project'; project.mkdir()
            (entry/'navigation_service.py').write_text('# deployed source\n')
            (entry/'navigation_settings.json').write_text('{"target_height_m":0.5}')
            binary = entry/'old_binary'; binary.write_bytes(b'old running binary')
            runtime = base/'actual-runtime.yaml'; runtime.write_text('actual: loaded\n')
            proc = base/'proc'; pid = proc/'123'; pid.mkdir(parents=True)
            (pid/'cmdline').write_bytes(os.fsencode(binary)+b'\0--params-file\0'+os.fsencode(runtime)+b'\0')
            (pid/'comm').write_text('fc01_system_node\n')
            (pid/'environ').write_bytes(b'ROS_DOMAIN_ID=7\0SECRET_TOKEN=hidden\0')
            (pid/'exe').symlink_to(binary); (pid/'cwd').symlink_to(entry)
            (pid/'fd').mkdir()
            calls = []
            def command(argv, timeout=4):
                calls.append(argv)
                if 'show' in argv:
                    return {'status':'ok','returncode':0,'stdout':
                        f'WorkingDirectory={entry}\nExecStart=/bin/bash {entry}/navigation_boot.sh run\n'
                        f'Environment="FC01_PROJECT_ROOT={project}" ROS_DOMAIN_ID=7 SECRET_TOKEN=hidden\n'
                        'MainPID=123\nActiveState=active\nDropInPaths=/etc/example.conf\n'}
                return {'status':'unknown','reason':'fixture tool unavailable'}
            before = {p:p.read_bytes() for p in (binary, runtime, entry/'navigation_settings.json')}
            report = m.collect('fc01-navigation.service', '/dev/ttyS2', command=command, proc_root=proc)
            self.assertEqual(report['service']['properties']['WorkingDirectory'], str(entry))
            self.assertEqual(report['service']['properties']['DropInPaths'], '/etc/example.conf')
            self.assertNotIn('hidden', json.dumps(report))
            self.assertEqual(report['processes']['items'][0]['environment']['ROS_DOMAIN_ID'], '7')
            self.assertEqual(report['processes']['items'][0]['executable']['sha256'],
                             hashlib.sha256(binary.read_bytes()).hexdigest())
            self.assertTrue(any(x['path']==str(runtime) for x in report['files']))
            self.assertTrue(any(x['path']==str(entry/'navigation_service.py') for x in report['files']))
            self.assertEqual(before, {p:p.read_bytes() for p in before})
            self.assertTrue(all(Path(x[0]).name in ('systemctl','fuser') for x in calls))
            self.assertFalse(any(x in {'start','stop','restart','enable','disable','daemon-reload'}
                                 for call in calls for x in call))
            self.assertEqual(report['ros']['status'], 'not_requested')

    def test_collect_persists_report_when_systemctl_is_missing(self):
        m = self.api()
        with tempfile.TemporaryDirectory() as d:
            base = Path(d); proc = base/'proc'; proc.mkdir()
            result = m.collect('fc01-navigation.service', '/missing-device', proc_root=proc,
                               command=lambda *a,**k:{'status':'unknown','reason':'missing'})
            self.assertEqual(result['service']['status'], 'unknown')
            self.assertEqual(result['serial']['status'], 'unknown')
            self.assertEqual(result['service']['properties'], {})

    def test_systemctl_success_does_not_mark_missing_unit_as_observed(self):
        m = self.api()
        with tempfile.TemporaryDirectory() as d:
            def command(argv, **kwargs):
                if 'show' in argv:
                    return dict(status='ok', stdout='LoadState=not-found\nMainPID=0\nActiveState=inactive\n')
                return dict(status='unknown', reason='unavailable')
            result = m.collect('missing.service', '/dev/ttyS2', proc_root=Path(d), command=command)
            self.assertEqual(result['service']['status'], 'unknown')
            self.assertIn('not-found', result['service']['reason'])

    def test_ros_domain_mismatch_is_reported_without_changing_environment(self):
        m = self.api()
        self.assertTrue(callable(getattr(m, 'compare_ros_environment', None)), 'missing domain comparison')
        report = dict(service=dict(properties=dict(Environment=dict(ROS_DOMAIN_ID='7'))),
                      processes=dict(items=[]), ros=dict(status='ok', environment=dict(ROS_DOMAIN_ID='0')))
        row = m.compare_ros_environment(report)
        self.assertEqual(row['ROS_DOMAIN_ID']['status'], 'mismatch')
        self.assertEqual(report['ros']['environment']['ROS_DOMAIN_ID'], '0')

    def test_missing_deployment_does_not_turn_default_domain_into_a_match(self):
        m = self.api()
        report = dict(service=dict(properties=dict(LoadState='not-found')),
                      processes=dict(items=[]), ros=dict(status='ok', environment={}))
        self.assertEqual(m.compare_ros_environment(report)['ROS_DOMAIN_ID']['status'], 'unknown')


if __name__ == '__main__':
    unittest.main()
