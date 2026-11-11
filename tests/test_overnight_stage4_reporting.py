"""报告来源归属回归：所有证据均为临时哨兵，命令读取器为替身，不操作服务。"""

from __future__ import annotations

import contextlib
import io
import json
from pathlib import Path
import shutil
import subprocess
import tempfile
import types
import unittest
from unittest.mock import patch
import zipfile

import navigation_report as report

ROOT = Path(__file__).resolve().parents[1]
BASELINE = 'f46468c21cac677dec0a3b300eb779de2fca44fd'


class ReportRuntimeSourceContainmentTests(unittest.TestCase):
    """保留显式入口语义，拒绝由status.json间接选择home外的运行入口。"""

    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='fc01_report_containment_')
        self.addCleanup(self.temp.cleanup)
        self.base = Path(self.temp.name)
        self.home = self.base / 'home'
        self.entry = self.home / 'invoked'
        self.folder = self.home / 'FC01_navigation'
        self.outside = self.base / 'outside'
        for path in (self.entry, self.folder, self.outside):
            path.mkdir(parents=True)
        self.marker = b'OUTSIDE_HOME_TEST_FIXTURE_ONLY'
        (self.outside / 'FC01_RELEASE.json').write_bytes(self.marker)
        self.calls = []

    def reader(self, argv, **options):
        # 真实收集器仍验证调用次序，但不执行systemctl或journalctl。
        self.calls.append((argv, options))
        return types.SimpleNamespace(stdout='fixture only', stderr='', returncode=0)

    def capture(self, source, *, entry=None):
        self.folder.joinpath('status.json').write_text(
            json.dumps({'config_source_path': str(source)})
        )
        with contextlib.redirect_stdout(io.StringIO()):
            target = report.collect_report(self.home, entry=entry or self.entry, runner=self.reader)
        with zipfile.ZipFile(target) as archive:
            files = {name: archive.read(name) for name in archive.namelist()}
        self.assertEqual(len(self.calls), 2)
        self.assertEqual(
            self.calls[0][0], ['/usr/bin/systemctl', 'status', report.SERVICE, '--no-pager']
        )
        self.assertEqual(
            self.calls[1][0],
            ['/usr/bin/journalctl', '-u', report.SERVICE, '-n', '100', '--no-pager'],
        )
        return files, json.loads(files['REPORT_CAPTURE.json'])

    def test_parent_traversal_does_not_capture_outside_home(self):
        source = self.home / '..' / 'outside/native/config/flight_test.yaml'
        files, capture = self.capture(source)
        self.assertNotIn('running_entry', capture['entries'])
        self.assertNotIn('running_entry/FC01_RELEASE.json', files)
        self.assertNotIn(self.marker, files.values())

    def test_parent_symlink_does_not_capture_outside_home(self):
        alias = self.home / 'alias'
        alias.symlink_to(self.outside, target_is_directory=True)
        files, capture = self.capture(alias / 'native/config/flight_test.yaml')
        self.assertNotIn('running_entry', capture['entries'])
        self.assertNotIn('running_entry/FC01_RELEASE.json', files)
        self.assertNotIn(self.marker, files.values())

    def test_normal_running_entry_inside_home_is_still_captured(self):
        running = self.home / 'running'
        running.mkdir()
        (running / 'FC01_RELEASE.json').write_bytes(b'INSIDE_HOME')
        files, capture = self.capture(running / 'native/config/flight_test.yaml')
        self.assertEqual(capture['entries']['running_entry'], str(running))
        self.assertEqual(files['running_entry/FC01_RELEASE.json'], b'INSIDE_HOME')

    def test_internal_directory_alias_keeps_original_entry_display(self):
        running = self.home / 'running'
        running.mkdir()
        (running / 'FC01_RELEASE.json').write_bytes(b'INSIDE_HOME_ALIAS')
        alias = self.home / 'alias'
        alias.symlink_to(running, target_is_directory=True)
        files, capture = self.capture(alias / 'native/config/flight_test.yaml')
        self.assertEqual(capture['entries']['running_entry'], str(alias))
        self.assertEqual(files['running_entry/FC01_RELEASE.json'], b'INSIDE_HOME_ALIAS')

    def test_explicit_invoked_entry_outside_home_keeps_existing_semantics(self):
        files, capture = self.capture(
            self.entry / 'native/config/flight_test.yaml', entry=self.outside
        )
        self.assertEqual(capture['entries']['invoked_entry'], str(self.outside))
        self.assertEqual(files['invoked_entry/FC01_RELEASE.json'], self.marker)

    def test_safe_entry_selection_matches_immutable_baseline(self):
        if shutil.which('git') is None:
            self.skipTest('历史选择规则差分需要Git；其他归属回归仍独立执行')
        source = subprocess.run(
            ['git', '-C', str(ROOT), 'show', BASELINE + ':navigation_report.py'],
            capture_output=True,
            text=True,
            check=False,
        )
        if source.returncode:
            self.skipTest('克隆缺少不可变f46468c对象；不能声称完成历史差分')
        before = types.ModuleType('fc01_stage4_original_report')
        exec(compile(source.stdout, str(ROOT / 'navigation_report.py'), 'exec'), before.__dict__)
        running = self.home / 'running'
        running.mkdir()
        alias = self.home / 'alias'
        alias.symlink_to(running, target_is_directory=True)
        rows = (
            {},
            {'config_source_path': 123},
            {'config_source_path': 'relative/native/config/flight_test.yaml'},
            {'config_source_path': str(self.entry / 'native/config/flight_test.yaml')},
            {'config_source_path': str(running / 'native/config/flight_test.yaml')},
            {'config_source_path': str(alias / 'native/config/flight_test.yaml')},
        )
        for entry in (self.entry, self.outside):
            for row in rows:
                with self.subTest(entry=entry, row=row):
                    self.assertEqual(
                        before._report_entries(self.home, entry, row),
                        report._report_entries(self.home, entry, row),
                    )

    def test_resolve_io_error_propagates_original_exception(self):
        running = self.home / 'running'
        failure = OSError('fixture canonical source unavailable')
        original = Path.resolve

        def resolve(path, *args, **kwargs):
            if path == running:
                raise failure
            return original(path, *args, **kwargs)

        with patch.object(Path, 'resolve', new=resolve):
            with self.assertRaises(OSError) as raised:
                report._report_entries(
                    self.home,
                    self.entry,
                    {'config_source_path': str(running / 'native/config/flight_test.yaml')},
                )
        self.assertIs(raised.exception, failure)

    def test_symlink_loop_does_not_turn_into_successful_report(self):
        loop = self.home / 'loop'
        loop.symlink_to(loop, target_is_directory=True)
        self.folder.joinpath('status.json').write_text(
            json.dumps({'config_source_path': str(loop / 'native/config/flight_test.yaml')})
        )
        original = report.os.fdopen
        outputs = []

        def fdopen(fd, mode, *args, **kwargs):
            stream = original(fd, mode, *args, **kwargs)
            if mode == 'wb':
                outputs.append(stream)
            return stream

        with patch.object(report.os, 'fdopen', side_effect=fdopen):
            with self.assertRaises(RuntimeError):
                with contextlib.redirect_stdout(io.StringIO()):
                    report.collect_report(self.home, entry=self.entry, runner=self.reader)
        self.assertEqual(len(outputs), 1)
        self.assertTrue(outputs[0].closed)
        self.assertEqual(self.calls, [])


if __name__ == '__main__':
    unittest.main()
