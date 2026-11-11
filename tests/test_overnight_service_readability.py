"""只用替身重放配置工厂：比较本轮原版与具名函数，不启动服务/ROS/Agent。"""

from __future__ import annotations

import ast
import contextlib
import io
from pathlib import Path
import shutil
import subprocess
import sys
import types
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
BASELINE = 'd8d14ff30be6bc89dce805205f6489795a5efa5e'


class ServiceFactoryEquivalenceTests(unittest.TestCase):
    """保留真实入口函数，替换外部I/O，只比较调用次序、返回对象、异常与闭包取值。"""

    @classmethod
    def setUpClass(cls):
        if shutil.which('git') is None:
            raise unittest.SkipTest('差分需要Git中的本轮不可变基线')
        probe = subprocess.run(
            ['git', '-C', str(ROOT), 'show', BASELINE + ':navigation_service.py'],
            capture_output=True,
            text=True,
            check=False,
        )
        if probe.returncode:
            raise unittest.SkipTest('导出源码或浅克隆缺少本轮基线；不能声称已完成历史差分')
        cls.original_source = probe.stdout
        cls.current_source = (ROOT / 'navigation_service.py').read_text()
        if str(ROOT) not in sys.path:
            sys.path.insert(0, str(ROOT))

    def load(self, source):
        # __file__仍指生产位置，让entry默认值与原版一致；__name__不为__main__，不执行CLI。
        module = types.ModuleType('fc01_service_readability_fixture')
        module.__file__ = str(ROOT / 'navigation_service.py')
        exec(compile(source, module.__file__, 'exec'), module.__dict__)
        return module

    def replay(self, source, *, fail_check=0, fail_load=0, signals=False):
        module = self.load(source)
        trace = []
        handlers = {}
        args = types.SimpleNamespace(mode=module.FLIGHT_TEST)
        counts = {'check': 0, 'load': 0}
        injected = OSError('controlled factory failure')
        original_entry = Path('/test/fc01-entry')
        module.ENTRY = original_entry

        def load_profile(root, agent, entry, mode, *, config_path):
            counts['load'] += 1
            trace.append(
                ['load', counts['load'], str(root), str(agent), str(entry), mode, str(config_path)]
            )
            if counts['load'] == fail_load:
                raise injected
            # 每次返回新对象；监督器必须收到本次结果本身，不能拿启动说明的profile代替。
            value = {
                'fc01_system': {
                    'ros__parameters': {
                        'aa_hover_thrust': 0.5,
                        'aa_takeoff_reserve_thrust': 0.03,
                        'rc_target_height': 1.5,
                        'flight_actions_enabled': True,
                        'arm_enabled': False,
                        'takeoff_enabled': True,
                        'rc_navigation_enabled': True,
                        'visual_navigation_enabled': False,
                    }
                },
                'load_number': counts['load'],
            }
            returned.append(value)
            return value

        def check_controller():
            counts['check'] += 1
            trace.append(['check', counts['check']])
            if counts['check'] == fail_check:
                raise injected
            return Path('/test/controller')

        def supervise(root, agent, folder, stopped, **options):
            # 先检查入口交出的参数。其余依赖均为替身，禁止真的启动任何服务或进程。
            trace.append(
                ['supervise', str(root), str(agent), str(folder), str(options['controller'])]
            )
            trace.append(['metadata', options['status_metadata']])
            trace.append(['config_path', str(options['config_path'])])
            trace.append(
                ['formatter_is_production', options['row_formatter'] is module.describe_row]
            )
            if signals:
                for signum, handler in handlers.items():
                    stopped.clear()
                    handler(signum, None)
                    trace.append(['stop_requested', int(signum), stopped.is_set()])
                return
            factory = options['config_factory']
            for attempt in range(1, 4):
                # 改同一args对象而非替换闭包，核对工厂每次读取当前mode；ENTRY也是运行时全局查询。
                args.mode = module.FLIGHT_TEST if attempt != 2 else module.DIAGNOSTIC
                module.ENTRY = original_entry / str(attempt)
                profile = factory(
                    Path('/test/root') / str(attempt), Path('/test/agent') / str(attempt)
                )
                trace.append(['factory_result', profile['load_number'], profile is returned[-1]])

        returned = []
        output = io.StringIO()
        with (
            patch.object(Path, 'home', return_value=Path('/test/home')),
            patch.object(
                module.signal,
                'signal',
                side_effect=lambda sig, callback: handlers.__setitem__(sig, callback),
            ),
            patch.object(module, 'load_profile', side_effect=load_profile),
            patch.object(module, 'check_controller', side_effect=check_controller),
            patch.object(module, 'controller_path', return_value=Path('/test/controller')),
            patch.object(module.standby, 'supervise', side_effect=supervise),
            contextlib.redirect_stdout(output),
        ):
            try:
                result = module._run_navigation_service(
                    args, Path('/test/root'), Path('/test/agent')
                )
                trace.append(['returned', result])
            except OSError as exc:
                # 既有错误对象必须原样传播；不能吞错、替换异常或继续加载配置。
                trace.append(['raised', type(exc).__name__, str(exc), exc is injected])
        return dict(trace=trace, stdout=output.getvalue(), counts=counts)

    def assert_replay_equal(self, **scenario):
        before = self.replay(self.original_source, **scenario)
        after = self.replay(self.current_source, **scenario)
        self.assertEqual(before, after)
        return after

    def test_repeated_factory_keeps_order_identity_and_live_closure(self):
        result = self.assert_replay_equal()
        self.assertEqual(result['counts'], {'check': 3, 'load': 4})
        business = [
            event[0] for event in result['trace'] if event[0] in ('load', 'check', 'factory_result')
        ]
        self.assertEqual(
            business,
            [
                'load',
                'check',
                'load',
                'factory_result',
                'check',
                'load',
                'factory_result',
                'check',
                'load',
                'factory_result',
            ],
        )

    def test_check_failure_stops_before_attempt_config_load(self):
        for attempt in (1, 2, 3):
            with self.subTest(attempt=attempt):
                result = self.assert_replay_equal(fail_check=attempt)
                self.assertEqual(result['counts'], {'check': attempt, 'load': attempt})
                self.assertEqual(
                    result['trace'][-1], ['raised', 'OSError', 'controlled factory failure', True]
                )

    def test_load_failure_preserves_original_exception(self):
        for call in (1, 2, 3, 4):
            with self.subTest(load=call):
                result = self.assert_replay_equal(fail_load=call)
                self.assertEqual(result['counts'], {'check': call - 1, 'load': call})
                self.assertEqual(
                    result['trace'][-1], ['raised', 'OSError', 'controlled factory failure', True]
                )

    def test_signal_callbacks_only_request_stop(self):
        result = self.assert_replay_equal(signals=True)
        self.assertEqual(result['counts'], {'check': 0, 'load': 1})
        self.assertEqual(
            sum(event[0] == 'stop_requested' and event[2] for event in result['trace']), 3
        )

    def test_ast_change_is_only_named_factory(self):
        # 把新函数机械还原成原lambda后，整份AST应相同；其他两份文件则直接AST相同。
        original = ast.parse(self.original_source)
        current = ast.parse(self.current_source)
        before_run = next(
            item
            for item in original.body
            if isinstance(item, ast.FunctionDef) and item.name == '_run_navigation_service'
        )
        after_run = next(
            item
            for item in current.body
            if isinstance(item, ast.FunctionDef) and item.name == '_run_navigation_service'
        )
        before_call = next(
            item.value
            for item in before_run.body
            if isinstance(item, ast.Expr)
            and isinstance(item.value, ast.Call)
            and isinstance(item.value.func, ast.Attribute)
            and item.value.func.attr == 'supervise'
        )
        after_call = next(
            item.value
            for item in after_run.body
            if isinstance(item, ast.Expr)
            and isinstance(item.value, ast.Call)
            and isinstance(item.value.func, ast.Attribute)
            and item.value.func.attr == 'supervise'
        )
        old_factory = next(
            item.value for item in before_call.keywords if item.arg == 'config_factory'
        )
        helper = next(
            item
            for item in after_run.body
            if isinstance(item, ast.FunctionDef) and item.name == 'checked_runtime_profile'
        )
        self.assertEqual([arg.arg for arg in helper.args.args], ['runtime_root', 'runtime_agent'])
        self.assertEqual(ast.dump(helper.body[0].value), ast.dump(old_factory.body.value.elts[0]))
        renamed_return = ast.parse(
            'load_profile(runtime_root, runtime_agent, ENTRY, args.mode, config_path=runtime_path)',
            mode='eval',
        ).body
        self.assertEqual(ast.dump(helper.body[1].value), ast.dump(renamed_return))
        after_run.body.remove(helper)
        next(item for item in after_call.keywords if item.arg == 'config_factory').value = (
            old_factory
        )
        self.assertEqual(ast.dump(original), ast.dump(current))
        for name in ('navigation_restore.py', 'standby.py'):
            with self.subTest(file=name):
                data = subprocess.check_output(
                    ['git', '-C', str(ROOT), 'show', BASELINE + ':' + name]
                )
                self.assertEqual(
                    ast.dump(ast.parse(data)), ast.dump(ast.parse((ROOT / name).read_bytes()))
                )


if __name__ == '__main__':
    unittest.main()
