"""比较真实 Evidence 的文件与归档字节；外部控制、ROS 和子进程均不参与。"""

import ast
import copy
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import types
import unittest
from unittest.mock import patch
import zipfile

import launcher

ROOT = Path(__file__).resolve().parents[1]
BASELINE = '8abc77b11dee5c754272b79618da455e482aba15'


class LauncherEvidenceReadabilityTests(unittest.TestCase):
    """摘要抽取只复用构造规则，写入时点、裁尾记录和失败处理必须保持。"""

    @classmethod
    def setUpClass(cls):
        if shutil.which('git') is None:
            raise unittest.SkipTest('证据差分需要不可变Git对象')
        result = subprocess.run(
            ['git', '-C', str(ROOT), 'show', BASELINE + ':launcher.py'],
            capture_output=True,
            text=True,
            check=False,
        )
        if result.returncode:
            raise unittest.SkipTest('导出/浅克隆缺少基线，不能声称完成差分')
        cls.source = result.stdout
        cls.before = types.ModuleType('fc01_stage4_evidence_original')
        cls.before.__file__ = str(ROOT / 'launcher.py')
        sys.modules[cls.before.__name__] = cls.before
        exec(compile(cls.source, cls.before.__file__, 'exec'), cls.before.__dict__)

    def replay(self, module, base, large):
        folder = base / 'run'
        if folder.exists():
            shutil.rmtree(folder)
        evidence = module.Evidence(folder, {'fixture': '中文证据', 'enabled': False})
        try:
            evidence.record({'kind': 'reply', 'seq': 1, 'accepted': False, 'message': 'FIRST'}, 1.0)
            evidence.record({'kind': 'state', 'phase': 'IDLE', 'armed': False}, 2.0)
            (folder / 'runtime.yaml').write_text('target_height: 1.5\n')
            data = b'controller log\n'
            if large:
                data += b'x' * (8 * 1024 * 1024) + b'END\n'
            (folder / 'controller.log').write_bytes(data)
            secret = base / 'unrelated'
            secret.write_text('fixture must not be archived')
            (folder / 'agent.log').symlink_to(secret)
            evidence.checkpoint('running')
            checkpoint = (folder / 'summary.json').read_bytes()
            target = evidence.report('finished')
            with zipfile.ZipFile(target) as archive:
                members = [(name, archive.read(name)) for name in archive.namelist()]
            return {
                'checkpoint': checkpoint,
                'members': members,
                'summary': (folder / 'summary.json').read_bytes(),
                'original_log': (folder / 'controller.log').read_bytes(),
                'first_failure': evidence.first_failure,
                'truncated': list(evidence.truncated),
            }
        finally:
            evidence.events.close()

    def test_checkpoint_and_archive_member_bytes_match_baseline(self):
        with tempfile.TemporaryDirectory(prefix='fc01-evidence-diff-') as temporary:
            for large in (False, True):
                with self.subTest(large=large):
                    base = Path(temporary)
                    before = self.replay(self.before, base, large)
                    after = self.replay(launcher, base, large)
                    self.assertEqual(before, after)
                    self.assertEqual(after['truncated'], ['controller.log'] if large else [])
                    self.assertNotIn('agent.log', [name for name, _ in after['members']])
                    summary = json.loads(after['summary'])
                    self.assertFalse(summary['hardware_flight_verified_by_this_bundle'])

    def test_checkpoint_io_failure_keeps_first_failure_and_call_values(self):
        results = []
        for module in (self.before, launcher):
            evidence = object.__new__(module.Evidence)
            evidence.first_failure = None
            evidence.last_state = {'phase': 'IDLE'}
            evidence.truncated = []
            calls = []
            failure = OSError('fixture checkpoint failure')

            def write(name, value):
                calls.append([name, copy.deepcopy(value)])
                raise failure

            with patch.object(evidence, 'write_json', side_effect=write):
                evidence.checkpoint('first')
                evidence.checkpoint('second')
            results.append((calls, evidence.first_failure))
        self.assertEqual(results[0], results[1])
        self.assertIn('fixture checkpoint failure', results[1][1]['message'])
        self.assertIsNone(results[1][0][0][1]['first_failure'])
        self.assertEqual(results[1][0][1][1]['first_failure'], results[1][1])

    def test_inlining_summary_restores_complete_baseline_ast(self):
        before = ast.parse(self.source)
        current = ast.parse((ROOT / 'launcher.py').read_text())
        klass = next(
            n for n in current.body if isinstance(n, ast.ClassDef) and n.name == 'Evidence'
        )
        helper = next(
            n for n in klass.body if isinstance(n, ast.FunctionDef) and n.name == '_summary'
        )
        self.assertEqual([arg.arg for arg in helper.args.args], ['self', 'outcome'])
        self.assertEqual(helper.args.defaults, [])
        self.assertEqual(helper.args.kwonlyargs, [])
        self.assertIsNone(helper.args.vararg)
        self.assertIsNone(helper.args.kwarg)
        self.assertEqual(helper.decorator_list, [])
        self.assertEqual(len(helper.body), 1)
        self.assertIsInstance(helper.body[0], ast.Return)
        expression = helper.body[0].value
        klass.body.remove(helper)
        replacements = []

        class InlineSummary(ast.NodeTransformer):
            def visit_Call(self, node):
                if (
                    isinstance(node.func, ast.Attribute)
                    and isinstance(node.func.value, ast.Name)
                    and node.func.value.id == 'self'
                    and node.func.attr == '_summary'
                ):
                    self_outer.assertEqual(len(node.args), 1)
                    self_outer.assertEqual(
                        ast.dump(node.args[0]), ast.dump(ast.Name(id='outcome', ctx=ast.Load()))
                    )
                    self_outer.assertEqual(node.keywords, [])
                    replacements.append(node)
                    return copy.deepcopy(expression)
                return self.generic_visit(node)

        self_outer = self
        current = InlineSummary().visit(current)
        self.assertEqual(len(replacements), 3)
        self.assertEqual(ast.dump(before), ast.dump(current))


if __name__ == '__main__':
    unittest.main()
