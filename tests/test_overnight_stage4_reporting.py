"""报告来源归属回归：所有证据均为临时哨兵，命令读取器为替身，不操作服务。"""

from __future__ import annotations

import contextlib
import ast
import copy
import importlib.util
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
REFACTOR_BASELINE = '9d0d4bccb15b0ad08eb77ffcee086bfc27c32e02'


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


class ReportReadabilityEquivalenceTests(unittest.TestCase):
    """比较独立功能修复之后的原报告与纯内存提取版本，不放行额外行为差异。"""

    @classmethod
    def setUpClass(cls):
        if shutil.which('git') is None:
            raise unittest.SkipTest('报告结构差分需要Git中的不可变功能修复基线')
        source = subprocess.run(
            ['git', '-C', str(ROOT), 'show', REFACTOR_BASELINE + ':navigation_report.py'],
            capture_output=True,
            text=True,
            check=False,
        )
        if source.returncode:
            raise unittest.SkipTest('克隆缺少9d0d4bc，不能声称已完成修复后结构差分')
        cls.original_source = source.stdout
        cls.before = types.ModuleType('fc01_stage4_report_after_functional_fix')
        exec(
            compile(source.stdout, str(ROOT / 'navigation_report.py'), 'exec'), cls.before.__dict__
        )
        spec = importlib.util.spec_from_file_location(
            'fc01_reporting_trace_fixture', ROOT / 'tests/stage2_python_trace.py'
        )
        cls.fixture = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cls.fixture)

    def replay(self, module, folder, *, status_bytes=None, decode_error=None):
        """重放有界尾部、缺文件和只读命令失败，比较归档顺序、字节及异常/流关闭。"""
        original_open = module.os.open
        original_fdopen = module.os.fdopen
        status_path = folder / 'home/FC01_navigation/status.json'
        replaced = False
        outputs = []

        def opening(path, *args, **kwargs):
            nonlocal replaced
            if status_bytes is not None and Path(path) == status_path and not replaced:
                replaced = True
                status_path.write_bytes(status_bytes)
                module.os.utime(status_path, ns=(1_000_000_000, 1_000_000_000))
            return original_open(path, *args, **kwargs)

        def fdopen(fd, mode, *args, **kwargs):
            stream = original_fdopen(fd, mode, *args, **kwargs)
            if mode == 'wb':
                outputs.append(stream)
            return stream

        with contextlib.ExitStack() as stack:
            stack.enter_context(patch.dict('sys.modules', {'navigation_report': module}))
            stack.enter_context(patch.object(module.os, 'open', side_effect=opening))
            stack.enter_context(patch.object(module.os, 'fdopen', side_effect=fdopen))
            if decode_error is not None:
                stack.enter_context(patch.object(module.json, 'loads', side_effect=decode_error))
            try:
                trace = self.fixture.report_trace(ROOT, folder)
                target = next((folder / 'home').glob('FC01_navigation_report_*.zip'))
                with zipfile.ZipFile(target) as archive:
                    members = [(name, archive.read(name)) for name in archive.namelist()]
                result = {'trace': trace, 'members': members}
            except (OSError, MemoryError) as error:
                result = {
                    'exception_type': type(error).__name__,
                    'exception_text': str(error),
                    'original_exception': error is decode_error,
                }
        result['archive_streams_closed'] = [stream.closed for stream in outputs]
        return result

    def assert_replay_equal(self, **scenario):
        with tempfile.TemporaryDirectory(prefix='fc01_report_refactor_') as temp:
            folder = Path(temp) / 'fixture'
            results = []
            for module in (self.before, report):
                folder.mkdir()
                results.append(self.replay(module, folder, **scenario))
                shutil.rmtree(folder)
        self.assertEqual(results[0], results[1])
        self.assertEqual(results[1]['archive_streams_closed'], [True])
        return results[1]

    def test_archive_bytes_order_and_readonly_calls_match_fixed_baseline(self):
        result = self.assert_replay_equal()
        self.assertEqual(len(result['trace']['commands']), 2)
        self.assertIn('REPORT_CAPTURE.json', [name for name, _ in result['members']])

    def test_unknown_status_variants_keep_exact_archive_evidence(self):
        for status in (b'\xff', b'[]', b'{"state":[],"controller_pid":7,"feedback_current":null}'):
            with self.subTest(status=status):
                self.assert_replay_equal(status_bytes=status)

    def test_unexpected_decode_errors_keep_identity_and_close_archive(self):
        for error in (OSError('fixture decode failure'), MemoryError('fixture allocation failure')):
            with self.subTest(error=type(error).__name__):
                result = self.assert_replay_equal(decode_error=error)
                self.assertTrue(result['original_exception'])

    def test_inline_normalization_proves_only_identity_block_was_extracted(self):
        before = ast.parse(self.original_source)
        current = ast.parse((ROOT / 'navigation_report.py').read_bytes())
        helper = next(
            node
            for node in current.body
            if isinstance(node, ast.FunctionDef) and node.name == '_record_status_identity'
        )
        self.assertEqual([arg.arg for arg in helper.args.args], ['capture', 'initial'])
        self.assertEqual(ast.dump(helper.body[-1]), ast.dump(ast.parse('return row').body[0]))
        collect = next(
            node
            for node in current.body
            if isinstance(node, ast.FunctionDef) and node.name == 'collect_report'
        )
        context = next(node for node in collect.body if isinstance(node, ast.With))
        assignment = next(
            node
            for node in context.body
            if isinstance(node, ast.Assign)
            and isinstance(node.value, ast.Call)
            and isinstance(node.value.func, ast.Name)
            and node.value.func.id == '_record_status_identity'
        )
        self.assertEqual(
            ast.dump(assignment),
            ast.dump(ast.parse('row = _record_status_identity(capture, initial)').body[0]),
        )
        index = context.body.index(assignment)
        context.body[index : index + 1] = copy.deepcopy(helper.body[:-1])
        current.body.remove(helper)
        self.assertEqual(ast.dump(before), ast.dump(current))


if __name__ == '__main__':
    unittest.main()
