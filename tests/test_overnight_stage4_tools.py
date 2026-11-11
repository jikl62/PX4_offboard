"""被动反馈探针生命周期故障注入：仅假 ROS、假时钟和内存文件，不连接设备。"""

import contextlib
import importlib.util
import io
import json
from pathlib import Path
import sys
import tempfile
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import patch


ROOT = Path(__file__).resolve().parents[1]


def load_probe(path=None):
    # 只加载模块定义；ROS 仍由生产 main 延迟导入，再由下方隔离夹具替换。
    source = path or ROOT / 'feedback_age_probe.py'
    spec = importlib.util.spec_from_file_location('fc01_stage4_feedback_probe', source)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class ProbeFixture:
    # 每个实例拥有自己的事件、默认 Context 和输出对象；故障点可抛同一个原异常对象。
    def __init__(self, failures=None, active=False):
        self.events = []
        self.failures = failures or {}
        self.active = active
        self.clock = 100.0
        self.subscriptions = []
        self.rows = []
        self.modules = {
            name: ModuleType(name)
            for name in ('rclpy', 'rclpy.node', 'rclpy.qos', 'px4_msgs', 'px4_msgs.msg')
        }
        owner = self

        class Output(io.StringIO):
            def write(self, value):
                owner.hit('output.write')
                return super().write(value)

            def flush(self):
                owner.hit('output.flush')
                return super().flush()

            def close(self):
                owner.events.append('output.close')
                super().close()
                owner.fail('output.close')

        self.output = Output()

        class Node:
            def __init__(self, name, **kwargs):
                owner.hit('node.create')
                owner.node_options = [name, kwargs]
                self.context = SimpleNamespace(get_domain_id=lambda: 77)

            def create_subscription(self, cls, topic, receive, qos):
                owner.hit('subscription.' + str(len(owner.subscriptions)))
                owner.subscriptions.append((cls.__name__, topic, receive, qos))
                return object()

            def count_publishers(self, topic):
                owner.hit('node.count_publishers')
                return 1

            def destroy_node(self):
                owner.hit('node.destroy')

        class Executor:
            def __init__(self):
                owner.hit('executor.create')

            def add_node(self, node):
                owner.hit('executor.add')

            def remove_node(self, node):
                owner.hit('executor.remove')

            def spin_once(self, *, timeout_sec):
                owner.events.append('spin.begin')
                owner.spin_wait = timeout_sec
                for _, _, receive, _ in owner.subscriptions:
                    receive(
                        SimpleNamespace(timestamp=200_000_000, timestamp_sample=199_999_000),
                        {'received_timestamp': 200_000_000_000},
                    )
                owner.clock += 5.1
                owner.fail('spin')
                owner.events.append('spin.end')

            def shutdown(self, *, timeout_sec):
                owner.events.append('executor.shutdown')
                owner.shutdown_wait = timeout_sec
                owner.fail('executor.shutdown')

        self.executor = Executor
        ros = self.modules['rclpy']
        ros.init = self.initialize
        ros.shutdown = self.shutdown
        ros.ok = lambda: self.active
        ros.get_rmw_implementation_identifier = lambda: 'fake_rmw'
        self.modules['rclpy.node'].Node = Node
        qos = self.modules['rclpy.qos']
        qos.QoSProfile = lambda **kwargs: kwargs
        qos.ReliabilityPolicy = SimpleNamespace(BEST_EFFORT=1)
        self.modules['px4_msgs'].__file__ = '/fixture/px4_msgs/__init__.py'
        for name in ('VehicleAttitude', 'VehicleLocalPosition'):
            setattr(self.modules['px4_msgs.msg'], name, type(name, (), {}))

    def fail(self, point):
        failure = self.failures.get(point)
        if callable(failure):
            failure()
        elif failure is not None:
            raise failure

    def hit(self, point):
        self.events.append(point)
        self.fail(point)

    def initialize(self, *, args):
        self.events.append('context.init')
        self.fail('context.before_init')
        if self.active:
            raise RuntimeError('Context.init() must only be called once')
        self.active = True
        self.fail('context.partial_init')

    def shutdown(self):
        self.events.append('context.shutdown')
        self.active = False
        self.fail('context.shutdown')

    def run(self, module=None):
        # output 只替换专有临时路径；控制器 status 仍读取不存在的临时路径，不接触用户运行数据。
        module = module or load_probe()
        real_open = Path.open
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / 'probe.jsonl'
            status = Path(directory) / 'missing-status.json'

            def open_file(path, *args, **kwargs):
                if path == output:
                    self.hit('output.open')
                    return self.output
                return real_open(path, *args, **kwargs)

            stdout = io.StringIO()
            try:
                with (
                    patch.dict(sys.modules, self.modules),
                    patch.object(module, 'ros_classes', return_value=self.executor),
                    patch.object(module.time, 'monotonic', side_effect=lambda: self.clock),
                    patch.object(module.time, 'time_ns', return_value=200_000_000_000),
                    patch.object(Path, 'open', open_file),
                    contextlib.redirect_stdout(stdout),
                ):
                    return module.main(
                        ['--duration', '5', '--status', str(status), '--output', str(output)]
                    )
            finally:
                self.rows = [json.loads(line) for line in stdout.getvalue().splitlines()]


def visible_chain(error):
    result = []
    while error is not None:
        if id(error) in [id(item) for item in result]:
            raise AssertionError('exception chain cycle')
        result.append(error)
        error = (
            error.__cause__
            if error.__cause__ is not None
            else (error.__context__ if not error.__suppress_context__ else None)
        )
    return result


class FeedbackProbeLifecycleTests(unittest.TestCase):
    def assert_released(self, fixture, expected=None):
        # 检查尝试顺序，不能把“尝试”升级为底层故障后仍必然成功的保证。
        cleanup = [
            event
            for event in fixture.events
            if event
            in {
                'executor.remove',
                'executor.shutdown',
                'node.destroy',
                'context.shutdown',
                'output.close',
            }
        ]
        self.assertEqual(
            cleanup,
            expected
            or [
                'executor.remove',
                'executor.shutdown',
                'node.destroy',
                'context.shutdown',
                'output.close',
            ],
        )
        self.assertTrue(fixture.output.closed)

    def test_normal_sampling_final_rows_and_resource_order(self):
        fixture = ProbeFixture()
        self.assertEqual(fixture.run(), 0)
        self.assertEqual(
            [row['kind'] for row in fixture.rows],
            ['feedback_age_probe_start', 'feedback_age_probe_final'],
        )
        self.assertEqual(fixture.rows[0]['ros_domain_id'], 77)
        self.assertFalse(fixture.rows[0]['control_publishers'])
        self.assertEqual(fixture.rows[1]['spin_calls_total'], 1)
        self.assertEqual(fixture.rows[1]['topics']['local']['callbacks_total'], 1)
        self.assertEqual(fixture.rows[1]['topics']['local']['last']['sample_us'], 199_999_000)
        self.assertEqual(
            fixture.node_options,
            [
                'fc01_feedback_age_probe',
                dict(
                    enable_rosout=False, start_parameter_services=False, use_global_arguments=False
                ),
            ],
        )
        self.assertEqual(
            [item[1] for item in fixture.subscriptions],
            ['/fmu/out/vehicle_attitude', '/fmu/out/vehicle_local_position'],
        )
        self.assertEqual(fixture.spin_wait, 0.01)
        self.assertEqual(fixture.shutdown_wait, 1.0)
        self.assert_released(fixture)
        self.assertFalse(fixture.active)

    def test_partial_init_failure_closes_only_acquired_context_and_file(self):
        error = RuntimeError('partial init')
        fixture = ProbeFixture({'context.partial_init': error})
        with self.assertRaises(RuntimeError) as caught:
            fixture.run()
        self.assertIs(caught.exception, error)
        self.assert_released(fixture, ['context.shutdown', 'output.close'])
        self.assertFalse(fixture.active)

    def test_init_failure_before_active_still_closes_output(self):
        error = RuntimeError('init unavailable')
        fixture = ProbeFixture({'context.before_init': error})
        with self.assertRaises(RuntimeError) as caught:
            fixture.run()
        self.assertIs(caught.exception, error)
        self.assert_released(fixture, ['output.close'])

    def test_existing_caller_context_survives_duplicate_init_failure(self):
        fixture = ProbeFixture(active=True)
        with self.assertRaisesRegex(RuntimeError, 'only be called once'):
            fixture.run()
        self.assertTrue(fixture.active)
        self.assert_released(fixture, ['output.close'])

    def test_node_constructor_failure_cannot_destroy_unreturned_object(self):
        fixture = ProbeFixture({'node.create': RuntimeError('node base')})
        with self.assertRaisesRegex(RuntimeError, 'node base'):
            fixture.run()
        self.assert_released(fixture, ['context.shutdown', 'output.close'])

    def test_executor_constructor_failure_still_destroys_node(self):
        fixture = ProbeFixture({'executor.create': RuntimeError('executor base')})
        with self.assertRaisesRegex(RuntimeError, 'executor base'):
            fixture.run()
        self.assert_released(fixture, ['node.destroy', 'context.shutdown', 'output.close'])

    def test_subscription_add_and_spin_failures_keep_original_error(self):
        for point in ('subscription.0', 'subscription.1', 'executor.add', 'spin'):
            with self.subTest(point=point):
                error = RuntimeError(point)
                fixture = ProbeFixture({point: error})
                with self.assertRaises(RuntimeError) as caught:
                    fixture.run()
                self.assertIs(caught.exception, error)
                self.assert_released(fixture)

    def test_each_cleanup_failure_does_not_skip_later_resources(self):
        for point in (
            'executor.remove',
            'executor.shutdown',
            'node.destroy',
            'context.shutdown',
            'output.close',
        ):
            with self.subTest(point=point):
                error = OSError(point)
                fixture = ProbeFixture({point: error})
                with self.assertRaises(OSError) as caught:
                    fixture.run()
                self.assertIs(caught.exception, error)
                self.assertEqual(visible_chain(error), [error])
                self.assert_released(fixture)

    def test_primary_explicit_cause_and_cleanup_order_remain_visible(self):
        primary = RuntimeError('original subscription')
        cause = ValueError('original cause')
        detail = LookupError('cause context')
        primary.__cause__ = cause
        cause.__context__ = detail
        cleanup = [OSError(name) for name in ('remove', 'shutdown', 'destroy', 'context', 'file')]
        failures = dict(
            zip(
                (
                    'executor.remove',
                    'executor.shutdown',
                    'node.destroy',
                    'context.shutdown',
                    'output.close',
                ),
                cleanup,
            )
        )
        failures['subscription.0'] = primary
        fixture = ProbeFixture(failures)
        with self.assertRaises(RuntimeError) as caught:
            fixture.run()
        self.assertIs(caught.exception, primary)
        self.assertIs(primary.__cause__, cause)
        self.assertIs(cause.__context__, detail)
        self.assertEqual(visible_chain(primary), [primary, cause, detail] + cleanup)
        self.assert_released(fixture)

    def test_original_implicit_context_is_not_hidden(self):
        original = KeyError('lookup origin')
        primary = RuntimeError('sampling')
        primary.__context__ = original
        cleanup = OSError('shutdown')
        fixture = ProbeFixture({'spin': primary, 'executor.shutdown': cleanup})
        with self.assertRaises(RuntimeError):
            fixture.run()
        self.assertIs(primary.__context__, original)
        self.assertEqual(visible_chain(primary), [primary, original, cleanup])
        self.assert_released(fixture)

    def test_no_primary_reports_first_cleanup_and_preserves_following_failures(self):
        first, second = OSError('first'), OSError('second')
        fixture = ProbeFixture({'executor.remove': first, 'node.destroy': second})
        with self.assertRaises(OSError) as caught:
            fixture.run()
        self.assertIs(caught.exception, first)
        self.assertEqual(visible_chain(first), [first, second])
        self.assert_released(fixture)

    def test_callback_keyboard_interrupt_keeps_original_final_output_and_130(self):
        fixture = ProbeFixture({'spin': KeyboardInterrupt()})
        self.assertEqual(fixture.run(), 130)
        self.assertTrue(fixture.rows[-1]['interrupted'])
        self.assertEqual(fixture.rows[-1]['kind'], 'feedback_age_probe_final')
        self.assert_released(fixture)

    def test_keyboard_interrupt_before_loop_and_during_cleanup_still_releases(self):
        for point in ('subscription.0', 'executor.remove'):
            with self.subTest(point=point):
                fixture = ProbeFixture({point: KeyboardInterrupt()})
                with self.assertRaises(KeyboardInterrupt):
                    fixture.run()
                self.assert_released(fixture)

    def test_external_context_shutdown_is_not_repeated(self):
        fixture = ProbeFixture()
        fixture.failures['spin'] = lambda: setattr(fixture, 'active', False)
        self.assertEqual(fixture.run(), 0)
        self.assert_released(
            fixture, ['executor.remove', 'executor.shutdown', 'node.destroy', 'output.close']
        )

    def test_write_and_flush_errors_keep_error_and_attempt_close(self):
        for point in ('output.write', 'output.flush'):
            with self.subTest(point=point):
                error = OSError(point)
                fixture = ProbeFixture({point: error})
                with self.assertRaises(OSError) as caught:
                    fixture.run()
                self.assertIs(caught.exception, error)
                self.assert_released(fixture)

    def test_early_observation_allocation_failure_still_closes_file(self):
        fixture = ProbeFixture()
        module = load_probe()
        # 输出文件已经取得，但尚未调用 ROS init；构造本地诊断缓存失败也必须关闭文件。
        with patch.object(
            module, 'TopicObservation', side_effect=MemoryError('observation allocation')
        ):
            with self.assertRaises(MemoryError):
                fixture.run(module)
        self.assert_released(fixture, ['output.close'])

    def test_repeated_starts_finish_with_no_owned_resources(self):
        for _ in range(20):
            fixture = ProbeFixture()
            self.assertEqual(fixture.run(), 0)
            self.assertFalse(fixture.active)
            self.assert_released(fixture)

    def test_reused_cleanup_exception_does_not_form_cycle_or_duplicate(self):
        original, reused = RuntimeError('sampling'), OSError('same object')
        fixture = ProbeFixture(
            {'spin': original, 'executor.remove': reused, 'node.destroy': reused}
        )
        with self.assertRaises(RuntimeError):
            fixture.run()
        self.assertEqual(visible_chain(original), [original, reused])
        self.assert_released(fixture)


if __name__ == '__main__':
    unittest.main()
