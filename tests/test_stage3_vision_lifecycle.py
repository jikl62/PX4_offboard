"""视觉工具资源故障注入：仅使用假 ROS，验证明确所有权和原始错误传播。"""

import signal
import sys
import traceback
import unittest
from unittest.mock import patch

from test_stage2_vision_shutdown import FakeRos, load_adapter


class VisionLifecycleTests(unittest.TestCase):
    """覆盖构造、初始化、接线、executor 和多项释放失败，不运行生产控制节点。"""

    def run_error(self, fixture, error):
        """运行实际入口并检查原异常对象、context 与两个信号处理器；返回异常因果链。"""
        with patch.dict(sys.modules, fixture.modules), self.assertRaises(type(error)) as caught:
            load_adapter().main([])
        self.assertIs(caught.exception, error)
        fixture.assert_handlers_restored(self)
        chain = []
        current = caught.exception
        while current is not None:
            self.assertNotIn(id(current), [id(item) for item in chain])
            chain.append(current)
            current = (
                current.__cause__
                if current.__cause__ is not None
                else (current.__context__ if not current.__suppress_context__ else None)
            )
        return chain

    def test_startup_configuration_failure_destroys_owned_node(self):
        """未配置身份时保留原拒绝，同时在 context 关闭前销毁已经成功构造的 Node。"""
        fixture = FakeRos()
        fixture.values['source_id'] = ''
        with patch.dict(sys.modules, fixture.modules), self.assertRaises(ValueError) as caught:
            load_adapter().main([])
        self.assertEqual(str(caught.exception), 'explicit source_id is required')
        self.assertEqual(
            fixture.events,
            ['context.init', 'node.create', 'node.destroy', 'context.shutdown'],
        )
        self.assertFalse(fixture.active)
        fixture.assert_handlers_restored(self)

    def test_resource_acquisition_failure_keeps_primary_and_releases_node(self):
        """发布器、两个订阅、定时器或启动日志失败均在交给 executor 前释放节点。"""
        for stage in ('publisher', 'subscription1', 'subscription2', 'timer', 'log'):
            with self.subTest(stage=stage):
                fixture = FakeRos()
                node = fixture.modules['rclpy.node'].Node
                error = MemoryError('stage3 allocation failure: ' + stage)
                subscription_count = 0

                def fail(*args, **kwargs):
                    """在明确的资源申请点注入同一个内存错误对象，不实际耗尽主机内存。"""
                    raise error

                def subscription(*args, **kwargs):
                    """允许先前订阅成功，检查第二个订阅失败时已获得的 Node 仍被销毁。"""
                    nonlocal subscription_count
                    subscription_count += 1
                    if stage == 'subscription' + str(subscription_count):
                        raise error
                    return object()

                method = {
                    'publisher': 'create_publisher',
                    'subscription1': 'create_subscription',
                    'subscription2': 'create_subscription',
                    'timer': 'create_timer',
                    'log': 'get_logger',
                }[stage]
                with patch.object(
                    node, method, subscription if stage.startswith('subscription') else fail
                ):
                    self.run_error(fixture, error)
                self.assertEqual(fixture.events[-2:], ['node.destroy', 'context.shutdown'])
                self.assertNotIn('executor.add', fixture.events)
                self.assertFalse(fixture.active)

    def test_init_failure_before_and_after_context_activation(self):
        """init 失败前无资源可清理；若已激活本入口 context，仍尝试关闭并保留初始错误。"""
        for activated in (False, True):
            with self.subTest(activated=activated):
                fixture = FakeRos()
                error = RuntimeError('stage3 initialization marker')

                def initialize(**kwargs):
                    """模拟 rclpy.init 在创建 context 前或创建后的信号安装阶段失败。"""
                    fixture.events.append('context.init.fail')
                    fixture.active = activated
                    raise error

                fixture.modules['rclpy'].init = initialize
                self.run_error(fixture, error)
                self.assertEqual(
                    fixture.events,
                    ['context.init.fail'] + (['context.shutdown'] if activated else []),
                )
                self.assertFalse(fixture.active)

    def test_failed_init_does_not_close_callers_active_context(self):
        """入口已有活动默认 context 时按 rclpy 原合同拒绝，不能关闭调用者的资源。"""
        fixture = FakeRos()
        fixture.active = True
        error = RuntimeError('Context.init() must only be called once')

        def initialize(**kwargs):
            """保留活动 context，模拟真实重复 init 的原始错误。"""
            fixture.events.append('context.init.fail')
            raise error

        fixture.modules['rclpy'].init = initialize
        self.run_error(fixture, error)
        self.assertTrue(fixture.active)
        self.assertEqual(fixture.events, ['context.init.fail'])

    def test_base_constructor_failure_has_no_application_owned_node(self):
        """ROS 基类尚未成功返回时不调用不完整对象的 destroy；仍释放本入口 context。"""
        fixture = FakeRos()
        error = MemoryError('stage3 base Node constructor marker')

        def initialize_node(*args, **kwargs):
            """在基类取得资源前失败；rclpy 内部构造异常的资源由其自身负责。"""
            fixture.events.append('node.create.fail')
            raise error

        with patch.object(fixture.modules['rclpy.node'].Node, '__init__', initialize_node):
            self.run_error(fixture, error)
        self.assertEqual(fixture.events, ['context.init', 'node.create.fail', 'context.shutdown'])
        self.assertFalse(fixture.active)

    def test_executor_acquisition_and_add_failures_release_owned_node(self):
        """获取 executor 或 add_node 失败均释放节点，已取得 executor 时还尝试 remove。"""
        for stage in ('get', 'add'):
            with self.subTest(stage=stage):
                fixture = FakeRos()
                error = RuntimeError('stage3 executor ' + stage + ' marker')

                def fail(*args):
                    """在 executor 所有权边界注入原异常对象，供精确传播断言。"""
                    fixture.events.append('executor.' + stage + '.fail')
                    raise error

                if stage == 'get':
                    fixture.modules['rclpy'].get_global_executor = fail
                else:
                    fixture.executor.add_node = fail
                self.run_error(fixture, error)
                expected = ['node.destroy', 'context.shutdown']
                if stage == 'add':
                    expected.insert(0, 'executor.remove')
                self.assertEqual(fixture.events[-len(expected) :], expected)
                self.assertFalse(fixture.active)

    def test_runtime_failure_is_not_masked_by_three_cleanup_failures(self):
        """主回调错误及 remove/destroy/shutdown 三项次生错误都可见，且全部动作被尝试。"""
        original_cause = ValueError('stage3 original inner marker')
        runtime_error = RuntimeError('stage3 callback marker')
        runtime_error.__cause__ = original_cause
        remove_error = OSError('stage3 remove marker')
        destroy_error = MemoryError('stage3 destroy marker')
        shutdown_error = RuntimeError('stage3 shutdown marker')
        fixture = FakeRos(
            spin_error=runtime_error,
            destroy_error=destroy_error,
            shutdown_error=shutdown_error,
        )

        def remove(node):
            """记录失败的移出动作；该错误不能阻止销毁节点和关闭 context。"""
            fixture.events.append('executor.remove')
            raise remove_error

        fixture.executor.remove_node = remove
        chain = self.run_error(fixture, runtime_error)
        self.assertEqual(
            chain, [runtime_error, original_cause, remove_error, destroy_error, shutdown_error]
        )
        self.assertEqual(
            fixture.events[-3:], ['executor.remove', 'node.destroy', 'context.shutdown']
        )
        self.assertFalse(fixture.active)
        rendered = ''.join(traceback.format_exception(type(runtime_error), runtime_error, None))
        for marker in ('inner', 'callback', 'remove', 'destroy', 'shutdown'):
            self.assertIn('stage3 ' + ('original ' if marker == 'inner' else '') + marker, rendered)

    def test_original_implicit_context_stays_visible_with_cleanup_failures(self):
        """业务错误来自 except 内的隐式 context 时，追加清理失败不能隐藏原业务首错。"""
        inner = ValueError('stage3 original implicit marker')
        primary = RuntimeError('stage3 outer implicit marker')
        primary.__context__ = inner
        cleanup = OSError('stage3 implicit cleanup marker')
        fixture = FakeRos(spin_error=primary, destroy_error=cleanup)
        self.assertEqual(self.run_error(fixture, primary), [primary, inner, cleanup])
        rendered = ''.join(traceback.format_exception(type(primary), primary, None))
        for marker in ('original implicit', 'outer implicit', 'implicit cleanup'):
            self.assertIn('stage3 ' + marker + ' marker', rendered)

    def test_cleanup_explicit_cause_back_to_primary_or_inner_is_cut(self):
        """清理错误显式指回主错误或原有 cause 时，仅剪回边，保留所有不同错误可见。"""
        for back_to_inner in (False, True):
            with self.subTest(back_to_inner=back_to_inner):
                inner = ValueError('stage3 original explicit marker')
                primary = RuntimeError('stage3 outer explicit marker')
                primary.__cause__ = inner
                cleanup = OSError('stage3 explicit cleanup marker')
                cleanup.__cause__ = inner if back_to_inner else primary
                fixture = FakeRos(spin_error=primary, destroy_error=cleanup)
                self.assertEqual(self.run_error(fixture, primary), [primary, inner, cleanup])
                rendered = ''.join(traceback.format_exception(type(primary), primary, None))
                for marker in ('original explicit', 'outer explicit', 'explicit cleanup'):
                    self.assertIn('stage3 ' + marker + ' marker', rendered)

    def test_normal_exit_propagates_first_cleanup_error_and_retains_secondary(self):
        """正常停止后若两项清理均失败，首项错误不被末项遮蔽，次项通过 cause 可见。"""
        destroy_error = OSError('stage3 first destroy marker')
        shutdown_error = RuntimeError('stage3 secondary shutdown marker')
        fixture = FakeRos(destroy_error=destroy_error, shutdown_error=shutdown_error)
        self.assertEqual(self.run_error(fixture, destroy_error), [destroy_error, shutdown_error])
        self.assertFalse(fixture.active)

    def test_cleanup_error_does_not_stop_restoring_other_signal_handler(self):
        """一个信号恢复 API 报错也继续恢复另一个；主错误及恢复错误均保持可见。"""
        primary = RuntimeError('stage3 callback before handler restoration marker')
        restore_error = OSError('stage3 handler restoration marker')
        fixture = FakeRos(spin_error=primary)
        original_signal = signal.signal
        restored = []

        def restore_then_fail(signum, handler):
            """实际先恢复处理器再注入故障，避免测试留下真实信号处理器变更。"""
            result = original_signal(signum, handler)
            if handler is fixture.previous_handlers[signum]:
                restored.append(signum)
                if signum == signal.SIGINT:
                    raise restore_error
            return result

        with patch.object(signal, 'signal', restore_then_fail):
            self.assertEqual(self.run_error(fixture, primary), [primary, restore_error])
        self.assertEqual(restored, [signal.SIGINT, signal.SIGTERM])

    def test_reused_cleanup_exception_object_does_not_create_cause_cycle(self):
        """多个释放点复用同一个错误对象时，原因链只有一次该对象，不形成循环。"""
        primary = RuntimeError('stage3 callback reused-error marker')
        cleanup = OSError('stage3 shared cleanup marker')
        fixture = FakeRos(spin_error=primary, destroy_error=cleanup, shutdown_error=cleanup)
        self.assertEqual(self.run_error(fixture, primary), [primary, cleanup])
        self.assertEqual(
            fixture.events[-3:], ['executor.remove', 'node.destroy', 'context.shutdown']
        )

    def test_stop_signals_during_startup_finish_before_cleanup(self):
        """构造和接线各阶段的双信号仅置停止标志；资源释放前不运行第一项回调。"""
        for method in ('__init__', 'create_publisher', 'create_subscription', 'create_timer'):
            with self.subTest(method=method):
                fixture = FakeRos()
                node = fixture.modules['rclpy.node'].Node
                original = getattr(node, method)

                def signal_then_return(*args, **kwargs):
                    """先完成当前同步申请，再发送重复停止请求；不在 C 回调间抛异常。"""
                    result = original(*args, **kwargs)
                    for signum in (signal.SIGINT, signal.SIGTERM):
                        signal.getsignal(signum)(signum, None)
                    return result

                with patch.object(node, method, signal_then_return):
                    with patch.dict(sys.modules, fixture.modules):
                        load_adapter().main([])
                self.assertNotIn('callback.begin', fixture.events)
                self.assertEqual(
                    fixture.events[-3:], ['executor.remove', 'node.destroy', 'context.shutdown']
                )
                self.assertFalse(fixture.active)
                fixture.assert_handlers_restored(self)

    def test_repeated_success_and_invalid_configuration_release_exactly_once(self):
        """同进程连续二十轮交替正常/失败启动，节点和本轮 context 每轮各释放一次。"""
        for cycle in range(20):
            fixture = FakeRos()
            invalid = bool(cycle % 2)
            if invalid:
                fixture.values['source_id'] = ''
            with self.subTest(cycle=cycle), patch.dict(sys.modules, fixture.modules):
                if invalid:
                    with self.assertRaises(ValueError):
                        load_adapter().main([])
                else:
                    load_adapter().main([])
            self.assertEqual(fixture.events.count('node.destroy'), 1)
            self.assertEqual(fixture.events.count('context.shutdown'), 1)
            self.assertFalse(fixture.active)
            fixture.assert_handlers_restored(self)


if __name__ == '__main__':
    unittest.main()
