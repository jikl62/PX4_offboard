# 只读 report 的现场证据回归：区分调用入口与实际运行入口，记录真实字节和抓取期间变化。
# 所有现场树均在临时 HOME；报告命令只经 injected reader，不能操作服务。
"""Read-only report capture keeps session identity and actual file bytes."""
import hashlib
import json
import os
from pathlib import Path
import tempfile
import types
import unittest
from unittest.mock import patch
import zipfile

import navigation_report as report


# 验证白名单、有界日志、来源身份、符号链接拒绝和非原子快照标记。
class NavigationReportTests(unittest.TestCase):
    # 刻意设置“新入口调用、旧入口运行”，并加入多个部署日志，防止报告只收当前源码目录。
    def setup_tree(self, directory):
        home = Path(directory)
        folder = home / 'FC01_navigation'
        folder.mkdir()
        entry = home / 'new'
        entry.mkdir()
        running = home / 'old'
        running.mkdir()
        row = dict(
            boot_id='test-boot',
            controller_pid=20,
            entry_version='old-version',
            config_source_path=str(running / 'native/config/flight_test.yaml'),
            state=dict(phase='SAFETY', reason='OPERATOR_TAKEOVER_LATCHED'),
        )
        (folder / 'status.json').write_text(json.dumps(row))
        (folder / 'controller.log').write_text('native trace\n')
        (folder / 'probe.json').write_text('{"boot_id":"another-boot"}')
        for root, marker in ((entry, 'new'), (running, 'old')):
            (root / 'FC01_RELEASE.json').write_text(json.dumps(dict(controller_build=marker)))
            (root / 'ARM64_BUILD_EVIDENCE.json').write_text(json.dumps(dict(real_build=marker)))
            (root / 'ACTIVATION_STATE.json').write_text(json.dumps(dict(activation=marker)))
            logs = folder / 'deploy-logs' / root.name
            logs.mkdir(parents=True)
            for number in range(8):
                path = logs / (str(number) + '.log')
                path.write_text(str(number))
                os.utime(path, ns=(number, number))
        return home, folder, entry, running, row

    # 返回无害的只读命令输出，供 collect_report 捕获而不调用真实服务。
    @staticmethod
    def reader(argv, **kwargs):
        return types.SimpleNamespace(stdout='read-only output', stderr='', returncode=0)

    # 报告需同时携带运行/调用两版证据、唯一归档项和逐文件 SHA，原目录字节不变。
    def test_capture_identifies_running_and_invoked_releases_without_mutation(self):
        with tempfile.TemporaryDirectory() as directory:
            home, folder, entry, running, row = self.setup_tree(directory)
            before = {str(p): p.read_bytes() for p in home.rglob('*') if p.is_file()}
            calls = []

            def runner(argv, **kwargs):
                calls.append(argv)
                return self.reader(argv, **kwargs)

            target = report.collect_report(home, entry=entry, runner=runner)
            self.assertEqual(target.stat().st_mode & 0o777, 0o600)
            with zipfile.ZipFile(target) as archive:
                files = {name: archive.read(name) for name in archive.namelist()}
                self.assertEqual(len(files), len(archive.namelist()))
            capture = json.loads(files['REPORT_CAPTURE.json'])
            self.assertEqual(capture['session']['controller_pid'], 20)
            self.assertEqual(
                capture['entries'], dict(invoked_entry=str(entry), running_entry=str(running))
            )
            self.assertEqual(
                json.loads(files['running_entry/FC01_RELEASE.json'])['controller_build'], 'old'
            )
            self.assertEqual(
                json.loads(files['invoked_entry/ARM64_BUILD_EVIDENCE.json'])['real_build'], 'new'
            )
            self.assertTrue(capture['non_atomic_live_capture'])
            self.assertFalse(capture['status_changed_during_capture'])
            self.assertEqual(len([name for name in files if name.startswith('deploy-logs/')]), 12)
            for name, item in capture['files'].items():
                if item['result'] == 'CAPTURED':
                    self.assertEqual(hashlib.sha256(files[name]).hexdigest(), item['sha256'])
            self.assertEqual(len(calls), 2)
            self.assertIn('status', calls[0])
            self.assertIn('journalctl', Path(calls[1][0]).name)
            for name, data in before.items():
                self.assertEqual(Path(name).read_bytes(), data)

    # 超长日志只取末尾且标记截断偏移，不能把尾部当成完整日志。
    def test_oversized_logs_are_bounded_and_tail_is_marked(self):
        with tempfile.TemporaryDirectory() as directory:
            home, folder, entry, _, _ = self.setup_tree(directory)
            (folder / 'controller.log').write_bytes(b'A' * 500 + b'TAIL')
            with patch.object(report, 'MAX_LOG', 100):
                target = report.collect_report(home, entry=entry, runner=self.reader)
            with zipfile.ZipFile(target) as archive:
                self.assertEqual(archive.read('controller.log'), b'A' * 96 + b'TAIL')
                item = json.loads(archive.read('REPORT_CAPTURE.json'))['files']['controller.log']
                self.assertEqual(item['tail_offset'], 404)
                self.assertTrue(item['truncated'])

    # 速度诊断快照及当前/轮转日志进入既有报告白名单，但不改变诊断控制权限。
    def test_velocity_command_diagnostics_are_captured_without_mutation(self):
        with tempfile.TemporaryDirectory() as directory:
            home, folder, entry, _, _ = self.setup_tree(directory)
            diagnostic_files = {
                'velocity_command_diagnostic.json': json.dumps(
                    dict(
                        control_effect='none',
                        source_id='test-session',
                        frame_id='body_frd',
                        sequence=7,
                        valid=False,
                        vx=0.2,
                        vy=-0.1,
                        vz=-0.3,
                    )
                ).encode(),
                'velocity_command_diagnostic.log': b'{"event":"invalid_command"}\n',
                'velocity_command_diagnostic.log.1': b'{"event":"receiver_started"}\n',
            }
            for name, data in diagnostic_files.items():
                (folder / name).write_bytes(data)
            target = report.collect_report(home, entry=entry, runner=self.reader)
            with zipfile.ZipFile(target) as archive:
                capture = json.loads(archive.read('REPORT_CAPTURE.json'))
                for name, data in diagnostic_files.items():
                    self.assertEqual(archive.read(name), data)
                    self.assertEqual((folder / name).read_bytes(), data)
                    item = capture['files'][name]
                    self.assertEqual(item['result'], 'CAPTURED')
                    self.assertEqual(item['sha256'], hashlib.sha256(data).hexdigest())

    # 新增诊断日志同样服从原有日志大小上限，不能绕过报告预算。
    def test_velocity_diagnostic_log_tail_retains_existing_size_limit(self):
        with tempfile.TemporaryDirectory() as directory:
            home, folder, entry, _, _ = self.setup_tree(directory)
            snapshot = b'{"diagnostic_only":true}'
            (folder / 'velocity_command_diagnostic.json').write_bytes(snapshot)
            for name in ('velocity_command_diagnostic.log', 'velocity_command_diagnostic.log.1'):
                (folder / name).write_bytes(b'X' * 500 + b'LAST')
            with patch.object(report, 'MAX_LOG', 100):
                target = report.collect_report(home, entry=entry, runner=self.reader)
            with zipfile.ZipFile(target) as archive:
                self.assertEqual(archive.read('velocity_command_diagnostic.json'), snapshot)
                capture = json.loads(archive.read('REPORT_CAPTURE.json'))
                for name in (
                    'velocity_command_diagnostic.log',
                    'velocity_command_diagnostic.log.1',
                ):
                    self.assertEqual(archive.read(name), b'X' * 96 + b'LAST')
                    self.assertEqual(capture['files'][name]['tail_offset'], 404)
                    self.assertTrue(capture['files'][name]['truncated'])

    # 白名单名称若变成链接也不能读取链接目标，避免泄露无关数据。
    def test_velocity_diagnostic_symlinks_are_not_collected(self):
        with tempfile.TemporaryDirectory() as directory:
            home, folder, entry, _, _ = self.setup_tree(directory)
            outside = home / 'not-a-diagnostic'
            outside.write_text('do not collect')
            names = (
                'velocity_command_diagnostic.json',
                'velocity_command_diagnostic.log',
                'velocity_command_diagnostic.log.1',
            )
            for name in names:
                (folder / name).symlink_to(outside)
            target = report.collect_report(home, entry=entry, runner=self.reader)
            with zipfile.ZipFile(target) as archive:
                capture = json.loads(archive.read('REPORT_CAPTURE.json'))
                for name in names:
                    self.assertNotIn(name, archive.namelist())
                    self.assertEqual(capture['files'][name]['result'], 'UNAVAILABLE')

    # 部分文件/命令缺失应记 UNAVAILABLE 并继续收其他证据，绝不沿日志链接取外部文件。
    def test_missing_metadata_and_symlink_logs_do_not_stop_capture(self):
        with tempfile.TemporaryDirectory() as directory:
            home, folder, entry, _, _ = self.setup_tree(directory)
            outside = home / 'not-a-controller-log'
            outside.write_text('do not collect')
            (folder / 'standby.log').symlink_to(outside)

            def failing(*args, **kwargs):
                raise FileNotFoundError('no systemd')

            target = report.collect_report(home, entry=entry, runner=failing)
            with zipfile.ZipFile(target) as archive:
                self.assertNotIn('standby.log', archive.namelist())
                capture = json.loads(archive.read('REPORT_CAPTURE.json'))
                self.assertEqual(capture['files']['standby.log']['result'], 'UNAVAILABLE')
                self.assertIn('error', capture['commands']['service.txt'])
                self.assertIn('no systemd', archive.read('service.txt').decode())

    # 抓取中修改状态，报告必须显式标记非原子与前后差异，不能声称同一时刻快照。
    def test_reports_capture_live_change_instead_of_claiming_atomic_snapshot(self):
        with tempfile.TemporaryDirectory() as directory:
            home, folder, entry, _, row = self.setup_tree(directory)
            original = os.open
            changed = False

            def changed_open(path, *args, **kwargs):
                nonlocal changed
                if Path(path) == folder / 'runtime.yaml' and not changed:
                    changed = True
                    row['state']['reason'] = 'NEW_FAULT'
                    (folder / 'status.json').write_text(json.dumps(row))
                return original(path, *args, **kwargs)

            with patch.object(report.os, 'open', side_effect=changed_open):
                target = report.collect_report(home, entry=entry, runner=self.reader)
            with zipfile.ZipFile(target) as archive:
                capture = json.loads(archive.read('REPORT_CAPTURE.json'))
                self.assertTrue(capture['status_changed_during_capture'])
                self.assertNotEqual(
                    archive.read('status.json'), archive.read('status_after_capture.json')
                )

    # 总预算不足时优先保留版本/构建身份和最终状态，不能让大日志挤掉判断依据。
    def test_full_log_budget_preserves_release_evidence_and_final_status(self):
        with tempfile.TemporaryDirectory() as directory:
            home, folder, entry, _, _ = self.setup_tree(directory)
            for name in report.BASE_FILES:
                if '.log' in name:
                    (folder / name).write_bytes(b'X' * 20000)
            with (
                patch.object(report, 'MAX_TOTAL', 8192),
                patch.object(report, 'MAX_LOG', 8192),
                patch.object(report, 'MAX_METADATA', 1024),
            ):
                target = report.collect_report(home, entry=entry, runner=self.reader)
            with zipfile.ZipFile(target) as archive:
                self.assertIn('invoked_entry/ARM64_BUILD_EVIDENCE.json', archive.namelist())
                self.assertIn('running_entry/FC01_RELEASE.json', archive.namelist())
                self.assertEqual(
                    archive.read('status.json'), archive.read('status_after_capture.json')
                )
                capture = json.loads(archive.read('REPORT_CAPTURE.json'))
                self.assertFalse(capture['status_changed_during_capture'])
                self.assertLessEqual(capture['captured_file_bytes'], 8192)

    # 没有抓到终态时 changed 必须未知，不能捏造“已变化”或“未变化”。
    def test_unavailable_final_status_is_unknown_instead_of_reported_changed(self):
        with tempfile.TemporaryDirectory() as directory:
            home, _, entry, _, _ = self.setup_tree(directory)
            with patch.object(report, 'MAX_TOTAL', 1):
                target = report.collect_report(home, entry=entry, runner=self.reader)
            with zipfile.ZipFile(target) as archive:
                capture = json.loads(archive.read('REPORT_CAPTURE.json'))
                self.assertIsNone(capture['status_changed_during_capture'])


if __name__ == '__main__':
    unittest.main()
