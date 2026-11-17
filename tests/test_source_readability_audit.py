#!/usr/bin/env python3
"""覆盖扫描器合同测试：SHA 绑定人工结论、逐路径排除与分组报告不能丢失来源。"""
import hashlib
import importlib.util
import json
from pathlib import Path
import subprocess
import tempfile
import unittest


# 直接导入只读工具，不执行其 CLI，也不读取或重写真实工程的 ledger/覆盖报告。
MODULE_PATH = Path(__file__).resolve().parents[1] / 'native/tools/source_readability_audit.py'
SPEC = importlib.util.spec_from_file_location('fc01_source_readability_audit', MODULE_PATH)
audit = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(audit)


class ReviewBindingTests(unittest.TestCase):
    def inspect(self, root, source, review=None, modified=()):
        path = root / 'controller.py'
        path.write_text(source, encoding='utf-8')
        return audit.inspect(
            root, 'controller.py', {'controller.py': review} if review else {}, set(modified)
        )

    def test_current_sha_accepts_explicit_review_and_legacy_fields(self):
        with tempfile.TemporaryDirectory() as directory:
            source = '# 中文说明\nvalue = 1\n'
            review = dict(
                sha256=hashlib.sha256(source.encode()).hexdigest(),
                annotation_complete=True,
                readability_complete=True,
                notes='已逐项审查',
                outstanding=[],
            )
            row = self.inspect(Path(directory), source, review, ['controller.py'])
            self.assertTrue(row['reviewed'])
            self.assertTrue(row['annotation_complete'])
            self.assertTrue(row['readability_complete'])
            # 兼容旧 ledger：同 SHA 的可读性完成标记可作为排版完成的默认值。
            self.assertTrue(row['layout_finished'])
            self.assertEqual(row['structural_refactor_pending'], [])
            self.assertTrue(row['preexisting_uncommitted_change'])
            self.assertFalse(row['stale_review'])

    def test_stale_sha_invalidates_all_completion_fields(self):
        with tempfile.TemporaryDirectory() as directory:
            review = dict(
                sha256=hashlib.sha256(b'value=1\n').hexdigest(),
                annotation_complete=True,
                readability_complete=True,
                layout_finished=True,
                structural_refactor_pending=['old function'],
            )
            row = self.inspect(Path(directory), '# 汉字很多也不表示已审查\nvalue=2\n', review)
            self.assertFalse(row['reviewed'])
            self.assertFalse(row['annotation_complete'])
            self.assertFalse(row['readability_complete'])
            self.assertFalse(row['layout_finished'])
            self.assertEqual(row['structural_refactor_pending'], [])
            self.assertTrue(row['stale_review'])

    def test_annotation_layout_and_structural_work_are_independent(self):
        with tempfile.TemporaryDirectory() as directory:
            source = 'value = 1\n'
            review = dict(
                sha256=hashlib.sha256(source.encode()).hexdigest(),
                annotation_complete=True,
                readability_complete=False,
                layout_finished=True,
                structural_refactor_pending=['split long callback'],
            )
            row = self.inspect(Path(directory), source, review)
            self.assertTrue(row['annotation_complete'])
            self.assertTrue(row['layout_finished'])
            self.assertFalse(row['readability_complete'])
            self.assertEqual(row['structural_refactor_pending'], ['split long callback'])

    def test_truthy_strings_cannot_claim_completed_work(self):
        with tempfile.TemporaryDirectory() as directory:
            source = 'value=1\n'
            review = dict(
                sha256=hashlib.sha256(source.encode()).hexdigest(),
                annotation_complete='true',
                readability_complete='yes',
                layout_finished=1,
            )
            row = self.inspect(Path(directory), source, review)
            self.assertFalse(row['annotation_complete'])
            self.assertFalse(row['readability_complete'])
            self.assertFalse(row['layout_finished'])

    def test_structural_pending_requires_explicit_list(self):
        with tempfile.TemporaryDirectory() as directory:
            source = 'value=1\n'
            review = dict(
                sha256=hashlib.sha256(source.encode()).hexdigest(),
                structural_refactor_pending='ambiguous text',
            )
            with self.assertRaises(ValueError):
                self.inspect(Path(directory), source, review)


class ScopeAndGroupingTests(unittest.TestCase):
    def test_historical_reproducers_are_excluded_but_current_tests_are_sources(self):
        names = (
            'native/tests/c_independent_repro/attack.cpp',
            'native/tests/c_s10_independent/attack.py',
            'native/tests/historical_r6/case.cpp',
            'docs/evidence/replay.cpp',
            'dist/installer.sh',
        )
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            for name in names:
                path = root / name
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text('# 原始证据\n')
                row = audit.inspect(root, name, {}, set())
                self.assertTrue(row['excluded'], name)
                self.assertNotIn('annotation_complete', row)
            for name in (
                'native/tests/test_current.cpp',
                'native/config/historical/candidate.yaml',
            ):
                path = root / name
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text('# 当前需要审查的文件\n')
                self.assertFalse(audit.inspect(root, name, {}, set())['excluded'], name)

    def test_only_registered_package_and_root_tool_metadata_are_candidates(self):
        expected = {
            'native/src/jikl_fc01_system/package.xml': 'package_xml',
            '.clang-format': 'formatter_config',
            'pyproject.toml': 'python_tool_config',
        }
        for name, language in expected.items():
            self.assertEqual(audit.language(Path(name)), language)
        for name in (
            'third_party/package.xml',
            'nested/.clang-format',
            'nested/pyproject.toml',
            'unknown.xml',
            'settings.json',
        ):
            self.assertIsNone(audit.language(Path(name)), name)

    def test_ignored_groups_preserve_every_path_and_original_reason(self):
        names = [
            '.superpowers/vision_replay/hash/vision_replay',
            '.superpowers/vision_replay/hash/vision_replay_parameters.inc',
            'build/pkg/output',
            'dist/bundle.sh',
            'module/__pycache__/a.pyc',
            'unknown/generated.py',
            'unknown/raw.bin',
        ]
        groups = audit.group_ignored(names + names[:2])
        flattened = [name for group in groups for name in group['paths']]
        self.assertCountEqual(flattened, names)
        self.assertEqual(len(flattened), len(set(flattened)))
        for group in groups:
            for name in group['paths']:
                original = audit.ignored_entry(name)
                self.assertEqual(
                    (group['category'], group['reason']), (original['category'], original['reason'])
                )
        self.assertEqual(groups, audit.group_ignored(reversed(names)))

    def test_thousands_of_generated_paths_fit_without_repeated_reason(self):
        # 复现报告膨胀的规模，仅在内存中构造测试路径，不生成真实工作区覆盖报告。
        names = [
            f'.superpowers/vision_replay/{index:020x}/vision_replay_parameters.inc'
            for index in range(3571)
        ]
        groups = audit.group_ignored(names)
        self.assertEqual(len(groups), 1)
        self.assertEqual(groups[0]['paths'], sorted(names))
        encoded = json.dumps(groups, ensure_ascii=False, indent=2).encode()
        self.assertLess(len(encoded), 1024 * 1024)


class ReportIntegrationTests(unittest.TestCase):
    def test_totals_metadata_and_unknown_ignored_source_remain_explicit(self):
        # 临时 Git 仓库隔离实际工程：这里只生成夹具报告对象，不落盘真实 ledger 或报告。
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)

            def git(*args):
                return subprocess.run(
                    ['git', '-C', str(root), *args], check=True, capture_output=True
                )

            git('init', '-q')
            files = {
                'controller.py': '# 中文说明\nvalue=1\n',
                'native/src/jikl_fc01_system/package.xml': '<package/>\n',
                '.clang-format': '# 格式规则\nBasedOnStyle: LLVM\n',
                'pyproject.toml': '# Python 排版\n[tool.black]\nline-length=100\n',
                '.gitignore': 'unknown/\n.superpowers/\n',
                'native/tests/c_independent_repro/old.cpp': '// 历史反例\n',
            }
            for name, source in files.items():
                path = root / name
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text(source)
            git('add', '.')
            git(
                '-c',
                'user.name=Test',
                '-c',
                'user.email=test@example.invalid',
                'commit',
                '-qm',
                'fixture',
            )
            commit = git('rev-parse', 'HEAD').stdout.decode().strip()
            for name in (
                'unknown/source.py',
                'unknown/raw.bin',
                '.superpowers/vision_replay/hash/cache.inc',
            ):
                path = root / name
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text('test fixture\n')
            ledger = {
                'baseline_commit': commit,
                'initially_modified': [],
                'reviews': {
                    'controller.py': {
                        'sha256': hashlib.sha256(files['controller.py'].encode()).hexdigest(),
                        'annotation_complete': True,
                        'readability_complete': False,
                        'layout_finished': True,
                        'structural_refactor_pending': ['long function'],
                    }
                },
            }
            result = audit.report(root, ledger)
            self.assertEqual(result['source_total'], 4)
            self.assertEqual(result['reviewed'], 1)
            self.assertEqual(result['annotation_complete'], 1)
            self.assertEqual(result['readability_complete'], 0)
            self.assertEqual(result['layout_finished'], 1)
            self.assertFalse(result['all_source_criteria_met'])
            self.assertCountEqual(
                result['annotation_unfinished'],
                ['native/src/jikl_fc01_system/package.xml', '.clang-format', 'pyproject.toml'],
            )
            self.assertEqual(
                result['unfinished'],
                sorted(
                    name
                    for name in files
                    if audit.language(Path(name)) and not audit.exclusion(Path(name))
                ),
            )
            self.assertEqual(
                result['structural_refactor_pending'],
                [{'path': 'controller.py', 'remaining': ['long function']}],
            )
            self.assertEqual(result['ignored_file_count'], 3)
            self.assertEqual(result['ignored_group_count'], 2)
            self.assertEqual(result['ignored_source_origin_review_required'], ['unknown/source.py'])
            flattened = [name for group in result['ignored_files'] for name in group['paths']]
            self.assertCountEqual(
                flattened,
                [
                    'unknown/source.py',
                    'unknown/raw.bin',
                    '.superpowers/vision_replay/hash/cache.inc',
                ],
            )
            for language in ('package_xml', 'formatter_config', 'python_tool_config'):
                self.assertEqual(result['by_language'][language]['total'], 1)
            self.assertTrue(
                all(not row['preexisting_uncommitted_change'] for row in result['files'])
            )


if __name__ == '__main__':
    unittest.main()
