"""只读速度诊断入口的离线生命周期故障注入，不创建 DDS 或飞控端点。"""

import importlib.util
from pathlib import Path
import signal
import sys
import traceback
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from test_stage2_vision_shutdown import FakeRos


SCRIPT = Path(__file__).resolve().parents[1] / 'native/tools/velocity_command_diagnostic.py'


def load_diagnostic():
    """加载本轮实际诊断源码；main 调用前不导入任何 ROS 模块。"""
    spec = importlib.util.spec_from_file_location('fc01_overnight_velocity_lifecycle', SCRIPT)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class DiagnosticRos(FakeRos):
    """复用既有退出夹具，补足真实速度诊断使用的时钟和消息类型。"""

    def __init__(self, **failures):
        super().__init__(**failures)
        owner = self
        node = self.modules['rclpy.node'].Node
        node.get_clock = lambda _: SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=100000000000)
        )
        node.count_publishers = lambda _node, _topic: 1
        self.modules['jikl_fc01_system.msg'].VelocityCommand = type('VelocityCommand', (), {})
        self.modules['rclpy'].spin = lambda _node: self.events.append('legacy.spin')
        self.init_options = []

        def initialize(*, args, signal_handler_options=None):
            """兼容修复前入口，以便同一故障注入测试先证实旧版缺少显式销毁。"""
            self.init_options.append(signal_handler_options)
            self.events.append('context.init')
            if owner.active:
                raise RuntimeError('Context.init() must only be called once')
            self.active = True

        self.modules['rclpy'].init = initialize


class DiagnosticLifecycleTests(unittest.TestCase):
    """每个失败点检查原异常对象、全部后续清理和信号恢复，而不只检查最终返回码。"""

    def run_error(self, fixture, error, *, module=None):
        """运行实际 main 并返回可见异常链；确保原对象不被清理错误替换。"""
        with patch.dict(sys.modules, fixture.modules), self.assertRaises(type(error)) as caught:
            (module or load_diagnostic()).main([])
        self.assertIs(caught.exception, error)
        fixture.assert_handlers_restored(self)
        chain = []
        current = error
        while current is not None:
            self.assertNotIn(id(current), [id(value) for value in chain])
            chain.append(current)
            current = (
                current.__cause__
                if current.__cause__ is not None
                else (current.__context__ if not current.__suppress_context__ else None)
            )
        return chain

    def test_configuration_failure_destroys_constructed_base_before_context(self):
        """基类已成功、参数被拒时仍应显式销毁 Node；旧完整构造赋值模式在此缺少 destroy。"""
        fixture = DiagnosticRos()
        fixture.values['topic'] = '/fmu/in/forbidden-diagnostic-test'
        with patch.dict(sys.modules, fixture.modules), self.assertRaises(ValueError) as caught:
            load_diagnostic().main([])
        self.assertEqual(
            str(caught.exception), 'invalid diagnostic topic, depth, or snapshot period'
        )
        self.assertEqual(
            fixture.events, ['context.init', 'node.create', 'node.destroy', 'context.shutdown']
        )
        self.assertFalse(fixture.active)
        fixture.assert_handlers_restored(self)

    def test_acquisition_failures_destroy_node_without_entering_executor(self):
        """参数、订阅、定时器、日志和首个快照失败都在 main 所有权下释放已建 Node。"""
        for stage in (
            'declare_parameter',
            'create_subscription',
            'create_timer',
            'get_logger',
            'get_clock',
        ):
            with self.subTest(stage=stage):
                fixture = DiagnosticRos()
                error = MemoryError('overnight startup ' + stage)

                def fail(*_args, **_kwargs):
                    raise error

                with patch.object(fixture.modules['rclpy.node'].Node, stage, fail):
                    self.run_error(fixture, error)
                self.assertEqual(fixture.events[-2:], ['node.destroy', 'context.shutdown'])
                self.assertNotIn('executor.add', fixture.events)
                self.assertFalse(fixture.active)

    def test_init_failure_cleans_only_context_activated_by_this_entry(self):
        """init 在激活前/后失败都保留原错；只在已取得活动 context 时关闭。"""
        for activated in (False, True):
            fixture = DiagnosticRos()
            error = RuntimeError('overnight partial init')

            def fail_init(**_kwargs):
                fixture.events.append('context.init.fail')
                fixture.active = activated
                raise error

            fixture.modules['rclpy'].init = fail_init
            with self.subTest(activated=activated):
                self.run_error(fixture, error)
                self.assertEqual(
                    fixture.events,
                    ['context.init.fail'] + (['context.shutdown'] if activated else []),
                )
                self.assertFalse(fixture.active)

    def test_existing_callers_context_is_not_closed_after_duplicate_init(self):
        """复用活动默认 context 的调用失败不能误关调用者 context。"""
        fixture = DiagnosticRos()
        fixture.active = True
        with (
            patch.dict(sys.modules, fixture.modules),
            self.assertRaisesRegex(RuntimeError, 'only be called once'),
        ):
            load_diagnostic().main([])
        self.assertEqual(fixture.events, ['context.init'])
        self.assertTrue(fixture.active)
        fixture.assert_handlers_restored(self)

    def test_base_node_failure_does_not_destroy_incomplete_object(self):
        """ROS 基类尚未返回时没有应用持有的完整 Node，只关闭本入口 context。"""
        fixture = DiagnosticRos()
        error = MemoryError('overnight base constructor')

        def fail_base(*_args, **_kwargs):
            fixture.events.append('node.create.fail')
            raise error

        with patch.object(fixture.modules['rclpy.node'].Node, '__init__', fail_base):
            self.run_error(fixture, error)
        self.assertEqual(fixture.events, ['context.init', 'node.create.fail', 'context.shutdown'])

    def test_normal_signals_finish_callback_then_finalize_remove_destroy_shutdown(self):
        """两个停止信号仅置标志；回调先返回，再写终态、移出、销毁、关闭，最后恢复原处理器。"""
        fixture = DiagnosticRos()
        module = load_diagnostic()
        original = module.finalize_diagnostic
        snapshots = []

        def finalize(*args):
            fixture.events.append('diagnostic.finalize')
            value = original(*args)
            snapshots.append(value)
            return value

        with (
            patch.dict(sys.modules, fixture.modules),
            patch.object(module, 'finalize_diagnostic', finalize),
        ):
            module.main([])
        self.assertEqual(
            [event for event in fixture.events if event != 'log'],
            [
                'context.init',
                'node.create',
                'executor.add',
                'callback.begin',
                'callback.end',
                'diagnostic.finalize',
                'executor.remove',
                'node.destroy',
                'context.shutdown',
            ],
        )
        self.assertFalse(snapshots[-1]['control_authority'])
        self.assertFalse(snapshots[-1]['receiver_running'])
        self.assertFalse(snapshots[-1]['cache_usable'])
        self.assertEqual(
            fixture.init_options, [fixture.modules['rclpy.signals'].SignalHandlerOptions.NO]
        )
        fixture.assert_handlers_restored(self)

    def test_runtime_and_all_cleanup_failures_keep_original_chain(self):
        """回调原错及既有cause优先，写终态/移出/销毁/关闭失败按尝试顺序全部可见。"""
        inner = ValueError('overnight original inner')
        primary = RuntimeError('overnight callback')
        primary.__cause__ = inner
        finalize_error = OSError('overnight finalize')
        remove_error = OSError('overnight remove')
        destroy_error = MemoryError('overnight destroy')
        shutdown_error = RuntimeError('overnight shutdown')
        fixture = DiagnosticRos(
            spin_error=primary, destroy_error=destroy_error, shutdown_error=shutdown_error
        )
        module = load_diagnostic()

        def fail_finalize(*_args):
            fixture.events.append('diagnostic.finalize')
            raise finalize_error

        def fail_remove(_node):
            fixture.events.append('executor.remove')
            raise remove_error

        fixture.executor.remove_node = fail_remove
        with patch.object(module, 'finalize_diagnostic', fail_finalize):
            chain = self.run_error(fixture, primary, module=module)
        self.assertEqual(
            chain, [primary, inner, finalize_error, remove_error, destroy_error, shutdown_error]
        )
        self.assertEqual(
            fixture.events[-4:],
            ['diagnostic.finalize', 'executor.remove', 'node.destroy', 'context.shutdown'],
        )
        rendered = ''.join(traceback.format_exception(type(primary), primary, None))
        for error in chain:
            self.assertIn(str(error), rendered)
        self.assertFalse(fixture.active)

    def test_normal_exit_exposes_first_cleanup_failure_and_later_errors(self):
        """没有业务异常时传播首个清理错误，后续错误不遮盖也不阻止其他释放。"""
        first = OSError('overnight normal destroy')
        later = RuntimeError('overnight normal shutdown')
        fixture = DiagnosticRos(destroy_error=first, shutdown_error=later)
        self.assertEqual(self.run_error(fixture, first), [first, later])
        self.assertEqual(
            fixture.events[-3:], ['executor.remove', 'node.destroy', 'context.shutdown']
        )

    def test_startup_signals_do_not_run_first_callback_or_skip_cleanup(self):
        """启动各资源边界中收到双信号后，先完成当前同步构造，再直接清理，不启动回调。"""
        for stage in (
            '__init__',
            'declare_parameter',
            'create_subscription',
            'create_timer',
            'get_logger',
        ):
            fixture = DiagnosticRos()
            node_type = fixture.modules['rclpy.node'].Node
            original = getattr(node_type, stage)

            def signal_then_return(*args, **kwargs):
                result = original(*args, **kwargs)
                for signum in (signal.SIGINT, signal.SIGTERM):
                    signal.getsignal(signum)(signum, None)
                return result

            with (
                self.subTest(stage=stage),
                patch.object(node_type, stage, signal_then_return),
                patch.dict(sys.modules, fixture.modules),
            ):
                load_diagnostic().main([])
            self.assertNotIn('callback.begin', fixture.events)
            self.assertEqual(
                fixture.events[-3:], ['executor.remove', 'node.destroy', 'context.shutdown']
            )
            fixture.assert_handlers_restored(self)

    def test_final_snapshot_filesystem_failure_is_visible_and_cleanup_continues(self):
        """真实写盘边界抛 OSError 时不吞错，仍移出/销毁节点并关闭本入口 context。"""
        module = load_diagnostic()
        fixture = DiagnosticRos()
        original = module.DiagnosticWriter.snapshot
        error = OSError('overnight terminal snapshot full disk')
        calls = 0

        def snapshot(writer, value):
            nonlocal calls
            calls += 1
            if not value.get('receiver_running', True):
                raise error
            return original(writer, value)

        with patch.object(module.DiagnosticWriter, 'snapshot', snapshot):
            self.assertEqual(self.run_error(fixture, error, module=module), [error])
        self.assertEqual(calls, 2)
        self.assertEqual(
            fixture.events[-3:], ['executor.remove', 'node.destroy', 'context.shutdown']
        )
        self.assertFalse(fixture.active)

    def test_executor_acquisition_and_registration_failures_release_owned_node(self):
        """取得 executor 或登记节点失败都保留原错，已经取得 executor 时仍尝试移出。"""
        for stage in ('get', 'add'):
            fixture = DiagnosticRos()
            error = RuntimeError('overnight executor ' + stage)

            def fail(*_args):
                fixture.events.append('executor.' + stage + '.fail')
                raise error

            if stage == 'get':
                fixture.modules['rclpy'].get_global_executor = fail
            else:
                fixture.executor.add_node = fail
            with self.subTest(stage=stage):
                self.run_error(fixture, error)
                expected = ['node.destroy', 'context.shutdown']
                if stage == 'add':
                    expected.insert(0, 'executor.remove')
                self.assertEqual(fixture.events[-len(expected) :], expected)
                self.assertFalse(fixture.active)

    def test_original_implicit_context_and_reused_cleanup_error_remain_acyclic(self):
        """原 implicit context 不被后续清理覆盖；多个清理重用同一错误时链中只保留一次。"""
        inner = ValueError('overnight original implicit')
        primary = RuntimeError('overnight outer implicit')
        primary.__context__ = inner
        cleanup = OSError('overnight shared cleanup')
        cleanup.__cause__ = inner
        fixture = DiagnosticRos(spin_error=primary, destroy_error=cleanup, shutdown_error=cleanup)
        self.assertEqual(self.run_error(fixture, primary), [primary, inner, cleanup])
        rendered = ''.join(traceback.format_exception(type(primary), primary, None))
        for error in (primary, inner, cleanup):
            self.assertIn(str(error), rendered)

    def test_external_shutdown_finishes_without_second_context_shutdown(self):
        """外部先关闭 context 并通知 executor 时仍销毁节点，不对已关闭 context 再次 shutdown。"""
        fixture = DiagnosticRos()
        stopped = fixture.modules['rclpy.executors'].ExternalShutdownException

        def external_stop(**_kwargs):
            fixture.events.append('context.external_shutdown')
            fixture.active = False
            raise stopped()

        fixture.executor.spin_once = external_stop
        with patch.dict(sys.modules, fixture.modules):
            load_diagnostic().main([])
        self.assertEqual(fixture.events[-2:], ['executor.remove', 'node.destroy'])
        self.assertNotIn('context.shutdown', fixture.events)
        fixture.assert_handlers_restored(self)

    def test_handler_restore_failure_still_restores_other_handler(self):
        """一项恢复处理器 API 出错也尝试另一项；先恢复真实处理器再注错，测试不污染宿主信号。"""
        primary = RuntimeError('overnight callback before signal restore')
        restore_error = OSError('overnight restore SIGINT')
        fixture = DiagnosticRos(spin_error=primary)
        original_signal = signal.signal
        restored = []

        def restore_then_fail(signum, handler):
            result = original_signal(signum, handler)
            if handler is fixture.previous_handlers[signum]:
                restored.append(signum)
                if signum == signal.SIGINT:
                    raise restore_error
            return result

        with patch.object(signal, 'signal', restore_then_fail):
            self.assertEqual(self.run_error(fixture, primary), [primary, restore_error])
        self.assertEqual(restored, [signal.SIGINT, signal.SIGTERM])

    def test_repeated_success_and_failed_configuration_release_once_each(self):
        """同进程交替二十轮正常/失败启动，每轮拥有的节点和 context 恰好释放一次。"""
        for cycle in range(20):
            fixture = DiagnosticRos()
            invalid = cycle % 2
            if invalid:
                fixture.values['qos_depth'] = 0
            with self.subTest(cycle=cycle), patch.dict(sys.modules, fixture.modules):
                if invalid:
                    with self.assertRaises(ValueError):
                        load_diagnostic().main([])
                else:
                    load_diagnostic().main([])
            self.assertEqual(fixture.events.count('node.destroy'), 1)
            self.assertEqual(fixture.events.count('context.shutdown'), 1)
            fixture.assert_handlers_restored(self)


if __name__ == '__main__':
    unittest.main()
