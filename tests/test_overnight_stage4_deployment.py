"""部署证据的临时资源归属回归；外部构建和动态库查询全部使用替身。"""

import ast
from contextlib import ExitStack, redirect_stdout
import copy
from datetime import datetime, timezone
import io
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import types
import unittest
from unittest.mock import patch

import navigation_build_evidence as evidence
from test_navigation_build_evidence import BuildFixture


class TemporaryEvidenceOwnershipTests(unittest.TestCase):
    """先存在的临时文件属于先前操作，创建失败不能把它当作本轮垃圾删除。"""

    def test_json_collision_preserves_existing_regular_file_and_symlink(self):
        for symlink in (False, True):
            with self.subTest(symlink=symlink), tempfile.TemporaryDirectory() as folder:
                destination = Path(folder) / 'evidence.json'
                destination.write_text('old complete evidence')
                temporary = Path(str(destination) + '.tmp-' + str(os.getpid()))
                target = Path(folder) / 'previous-record'
                target.write_text('previous temporary evidence')
                if symlink:
                    temporary.symlink_to(target)
                else:
                    temporary.write_bytes(target.read_bytes())
                with self.assertRaises(FileExistsError):
                    evidence.atomic_json(destination, {'new': True})
                self.assertEqual(temporary.read_text(), 'previous temporary evidence')
                self.assertEqual(temporary.is_symlink(), symlink)
                self.assertEqual(target.read_text(), 'previous temporary evidence')
                self.assertEqual(destination.read_text(), 'old complete evidence')

    def test_json_replace_failure_removes_owned_temporary_and_preserves_error(self):
        with tempfile.TemporaryDirectory() as folder:
            destination = Path(folder) / 'evidence.json'
            destination.write_text('old complete evidence')
            temporary = Path(str(destination) + '.tmp-' + str(os.getpid()))
            error = OSError('injected replace failure')
            with patch.object(evidence.os, 'replace', side_effect=error):
                with self.assertRaises(OSError) as caught:
                    evidence.atomic_json(destination, {'new': True})
            self.assertIs(caught.exception, error)
            self.assertFalse(temporary.exists())
            self.assertEqual(destination.read_text(), 'old complete evidence')

    def test_json_serialization_failure_removes_owned_temporary(self):
        with tempfile.TemporaryDirectory() as folder:
            destination = Path(folder) / 'evidence.json'
            temporary = Path(str(destination) + '.tmp-' + str(os.getpid()))
            with self.assertRaises(TypeError):
                evidence.atomic_json(destination, {'unsupported': object()})
            self.assertFalse(temporary.exists())
            self.assertFalse(destination.exists())


class TemporaryInstallLinkOwnershipTests(unittest.TestCase):
    """使用原有 fake CMake 夹具，仅在临时发布树里走真实 build_release 协调代码。"""

    def setUp(self):
        self.resources = ExitStack()
        self.addCleanup(self.resources.close)
        folder = self.resources.enter_context(tempfile.TemporaryDirectory())
        self.fixture = BuildFixture(folder)
        for replacement in self.fixture.patches():
            self.resources.enter_context(replacement)

    def build(self):
        return evidence.build_release(self.fixture.entry, self.fixture.sdk)

    def test_link_collision_preserves_existing_regular_file_and_symlink(self):
        first = self.build()
        saved = (self.fixture.entry / 'ARM64_BUILD_EVIDENCE.json').read_bytes()
        self.fixture.dependencies['trees'][0]['sha256'] = '2' * 64
        temporary = self.fixture.entry / ('.install-new-' + str(os.getpid()))
        target = self.fixture.entry / 'retained-link-target'
        target.mkdir()
        for symlink in (False, True):
            with self.subTest(symlink=symlink):
                if symlink:
                    temporary.symlink_to(target, target_is_directory=True)
                else:
                    temporary.write_text('previous operation')
                with self.assertRaises(FileExistsError):
                    self.build()
                self.assertEqual(temporary.is_symlink(), symlink)
                if symlink:
                    self.assertEqual(temporary.resolve(), target)
                else:
                    self.assertEqual(temporary.read_text(), 'previous operation')
                self.assertEqual(
                    (self.fixture.entry / 'install').resolve(), Path(first['install_prefix'])
                )
                self.assertEqual(
                    (self.fixture.entry / 'ARM64_BUILD_EVIDENCE.json').read_bytes(), saved
                )
                temporary.unlink()
        self.assertEqual(len(self.fixture.calls), 9)

    def test_link_replace_failure_removes_owned_link_and_preserves_error(self):
        first = self.build()
        saved = (self.fixture.entry / 'ARM64_BUILD_EVIDENCE.json').read_bytes()
        self.fixture.dependencies['trees'][0]['sha256'] = '3' * 64
        temporary = self.fixture.entry / ('.install-new-' + str(os.getpid()))
        error = OSError('injected install replace failure')
        with patch.object(evidence.os, 'replace', side_effect=error):
            with self.assertRaises(OSError) as caught:
                self.build()
        self.assertIs(caught.exception, error)
        self.assertFalse(temporary.exists())
        self.assertFalse(temporary.is_symlink())
        self.assertEqual((self.fixture.entry / 'install').resolve(), Path(first['install_prefix']))
        self.assertEqual((self.fixture.entry / 'ARM64_BUILD_EVIDENCE.json').read_bytes(), saved)
        self.assertEqual(json.loads(saved)['install_prefix'], first['install_prefix'])
        self.assertEqual(len(self.fixture.calls), 6)


class DeploymentReadabilityEquivalenceTests(unittest.TestCase):
    """从不可变Git基线取原协调器，比较真实执行轨迹；所有构建命令都被替身截获。"""

    @classmethod
    def setUpClass(cls):
        cls.root = Path(__file__).resolve().parents[1]
        cls.sources = {}
        # d66b905仅含已独立确认的资源归属修复；本轮纯提取相对于该点保持行为。
        for name in (
            'navigation_build_evidence.py',
            'navigation_install.py',
            'navigation_deploy.py',
        ):
            result = subprocess.run(
                ['git', '-C', str(cls.root), 'show', 'd66b905:' + name],
                capture_output=True,
                text=True,
                check=True,
            )
            cls.sources[name] = result.stdout

    def load_original(self):
        module = types.ModuleType('fc01_original_build_coordinator')
        module.__file__ = str(self.root / 'navigation_build_evidence.py')
        exec(
            compile(self.sources['navigation_build_evidence.py'], module.__file__, 'exec'),
            module.__dict__,
        )
        return module

    def test_install_and_deploy_preserve_recorded_behavior_baselines(self):
        for name in ('navigation_install.py', 'navigation_deploy.py'):
            with self.subTest(file=name):
                before = ast.parse(self.sources[name])
                after = ast.parse((self.root / name).read_text())
                if name == 'navigation_deploy.py':
                    # Stage6单独修复了DeploymentLog写失败后的关闭责任，不能再声称整文件行为等价。
                    # 先将完整现文件锁到该不可变修复提交，避免随当前工作树任意更新预期。
                    repaired = subprocess.run(
                        [
                            'git',
                            '-C',
                            str(self.root),
                            'show',
                            '6c4cc7d:navigation_deploy.py',
                        ],
                        capture_output=True,
                        text=True,
                        check=True,
                    )
                    self.assertEqual(ast.dump(ast.parse(repaired.stdout)), ast.dump(after))
                    # 只有已复现并独立测试的日志类允许变化；外部offline/online/main及全部接线
                    # 继续与Stage4起点逐项比较，不跳过部署编排或放宽安全行为的差分。
                    old_log = next(
                        item
                        for item in before.body
                        if isinstance(item, ast.ClassDef) and item.name == 'DeploymentLog'
                    )
                    new_log = next(
                        item
                        for item in after.body
                        if isinstance(item, ast.ClassDef) and item.name == 'DeploymentLog'
                    )
                    for method in ('record', 'step'):
                        self.assertEqual(
                            ast.dump(
                                next(
                                    item
                                    for item in old_log.body
                                    if isinstance(item, ast.FunctionDef) and item.name == method
                                )
                            ),
                            ast.dump(
                                next(
                                    item
                                    for item in new_log.body
                                    if isinstance(item, ast.FunctionDef) and item.name == method
                                )
                            ),
                        )
                    before.body.remove(old_log)
                    after.body.remove(new_log)
                self.assertEqual(ast.dump(before), ast.dump(after))

    def test_build_only_extracts_the_original_command_expression(self):
        before = ast.parse(self.sources['navigation_build_evidence.py'])
        after = ast.parse((self.root / 'navigation_build_evidence.py').read_text())
        old_function = next(
            node
            for node in before.body
            if isinstance(node, ast.FunctionDef) and node.name == 'build_release'
        )
        commands = next(
            node
            for node in ast.walk(old_function)
            if isinstance(node, ast.Assign)
            and any(
                isinstance(target, ast.Name) and target.id == 'commands' for target in node.targets
            )
        )
        helper = next(
            node
            for node in after.body
            if isinstance(node, ast.FunctionDef) and node.name == '_native_build_commands'
        )
        self.assertEqual(
            [item.arg for item in helper.args.args], ['entry', 'build', 'prefix', 'compiler']
        )
        self.assertFalse(helper.args.defaults)
        self.assertFalse(helper.decorator_list)
        self.assertEqual(len(helper.body), 1)
        self.assertIsInstance(helper.body[0], ast.Return)
        self.assertEqual(ast.dump(helper.body[0].value), ast.dump(commands.value))
        after.body.remove(helper)
        new_function = next(
            node
            for node in after.body
            if isinstance(node, ast.FunctionDef) and node.name == 'build_release'
        )
        call = next(
            node
            for node in ast.walk(new_function)
            if isinstance(node, ast.Assign)
            and any(
                isinstance(target, ast.Name) and target.id == 'commands' for target in node.targets
            )
        )
        self.assertEqual(
            ast.dump(call.value),
            ast.dump(
                ast.parse(
                    '_native_build_commands(entry, build, prefix, compiler)', mode='eval'
                ).body
            ),
        )
        call.value = copy.deepcopy(commands.value)
        self.assertEqual(ast.dump(before), ast.dump(after))

    def replay(self, module, folder, scenario):
        fixture = BuildFixture(folder)
        events = []
        injected = OSError('controlled build command failure')
        input_observations = 0
        allocations = 0

        def dependencies(*_):
            nonlocal input_observations
            input_observations += 1
            result = copy.deepcopy(fixture.dependencies)
            if scenario == 'inputs_changed' and input_observations > 1:
                result['trees'][0]['sha256'] = '4' * 64
            return result

        def command(argv, **kwargs):
            events.append([list(argv), dict(kwargs)])
            if scenario == 'fail_' + argv[1]:
                raise injected
            return fixture.run(argv, **kwargs)

        def allocate(*, prefix, dir):
            nonlocal allocations
            allocations += 1
            path = Path(dir) / ('deterministic-stage-' + str(allocations))
            path.mkdir()
            return str(path)

        if scenario == 'foreign_directory':
            (fixture.entry / 'install').mkdir()
        if scenario == 'foreign_link':
            (fixture.entry / 'install').symlink_to(fixture.sdk, target_is_directory=True)
        output = io.StringIO()
        result = failure = None
        with (
            patch.object(module.platform, 'machine', return_value='aarch64'),
            patch.object(module, 'dependency_snapshot', side_effect=dependencies),
            patch.object(module, 'resolved_libraries', side_effect=fixture.libraries),
            patch.object(module.subprocess, 'run', side_effect=command),
            patch.object(module.tempfile, 'mkdtemp', side_effect=allocate),
            patch.object(
                module,
                'datetime',
                types.SimpleNamespace(now=lambda _: datetime(2026, 11, 12, tzinfo=timezone.utc)),
            ),
            redirect_stdout(output),
        ):
            try:
                result = module.build_release(fixture.entry, fixture.sdk)
                if scenario == 'reuse':
                    result = module.build_release(fixture.entry, fixture.sdk)
            except (OSError, module.BuildEvidenceError) as error:
                failure = [type(error).__name__, str(error), error is injected]
        install = fixture.entry / 'install'
        evidence_path = fixture.entry / 'ARM64_BUILD_EVIDENCE.json'
        return dict(
            events=events,
            result=result,
            failure=failure,
            output=output.getvalue(),
            install_target=str(install.resolve()) if install.is_symlink() else None,
            evidence=evidence_path.read_bytes() if evidence_path.is_file() else None,
            retained_files=sorted(
                str(path.relative_to(fixture.entry)) for path in fixture.entry.rglob('*')
            ),
        )

    def test_full_coordinator_traces_match_for_reuse_failures_and_input_changes(self):
        scenarios = (
            'fresh',
            'reuse',
            'fail_-S',
            'fail_--build',
            'fail_--install',
            'inputs_changed',
            'foreign_directory',
            'foreign_link',
        )
        original = self.load_original()
        for scenario in scenarios:
            with self.subTest(scenario=scenario), tempfile.TemporaryDirectory() as temporary:
                folder = Path(temporary) / 'same-owned-fixture'
                folder.mkdir()
                before = self.replay(original, folder, scenario)
                # 只销毁本测试创建的临时夹具，再在完全相同路径重放，避免路径参与摘要造成假差异。
                shutil.rmtree(folder)
                folder.mkdir()
                after = self.replay(evidence, folder, scenario)
                self.assertEqual(before, after)


if __name__ == '__main__':
    unittest.main()
