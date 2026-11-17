"""被动采集器异常清理：只有假 ROS 与合成时钟，不创建进程、端口或设备。"""

import errno
import importlib.util
import os
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import patch


ROOT = Path(__file__).resolve().parents[1]


def load_collector():
    # 环境变量仅供本测试重放不可变基线源码；生产采集器不读取它。
    source = Path(os.environ.get('FC01_STAGE6_COLLECT_SOURCE', ROOT / 'navigation_collect.py'))
    spec = importlib.util.spec_from_file_location('stage6_passive_collector', source)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def visible_chain(error):
    """按 traceback 实际可见的 cause/context 顺序检查；出现环立即使测试失败。"""
    result = []
    while error is not None:
        if any(item is error for item in result):
            raise AssertionError('exception chain cycle')
        result.append(error)
        error = error.__cause__ or (None if error.__suppress_context__ else error.__context__)
    return result


class CollectFixture:
    """持有独立默认 Context 和 Node 替身，精确记录取得/释放顺序及原异常身份。"""

    def __init__(self, failures=None, *, active=False, partial_init=False):
        self.failures = failures or {}
        self.active = active
        self.partial_init = partial_init
        self.events = []
        self.nodes = []
        self.clock = 100.0
        self.modules = {
            name: ModuleType(name)
            for name in ('rclpy', 'rclpy.qos', 'rclpy.serialization', 'px4_msgs', 'px4_msgs.msg')
        }
        self.modules['px4_msgs'].__path__ = []
        messages = self.modules['px4_msgs.msg']
        messages.__file__ = '/synthetic/px4_msgs/msg/__init__.py'
        for type_name in load_collector().TOPICS.values():
            setattr(messages, type_name, type(type_name, (), {}))
        self.modules['px4_msgs'].msg = messages
        ros = self.modules['rclpy']
        ros.init, ros.ok, ros.shutdown = self.initialize, self.ok, self.shutdown
        ros.create_node, ros.spin_once = self.create_node, self.spin
        self.modules['rclpy.qos'].qos_profile_sensor_data = object()
        self.modules['rclpy.serialization'].serialize_message = lambda message: b'fixture-cdr'

    def fail(self, point):
        error = self.failures.get(point)
        if error is not None:
            raise error

    def ok(self):
        self.fail('ok')
        return self.active

    def initialize(self, *, args):
        self.events.append('init')
        if self.active:
            raise RuntimeError('borrowed Context already initialized')
        if self.partial_init:
            self.active = True
        self.fail('init')
        self.active = True

    def shutdown(self):
        self.events.append('shutdown')
        self.active = False
        self.fail('shutdown')

    def create_node(self, name):
        self.events.append('create')
        self.fail('create')
        owner = self

        class Node:
            def __init__(self):
                self.callbacks = []
                self.destroyed = False

            def create_subscription(self, cls, topic, receive, qos):
                owner.events.append('subscribe')
                owner.fail('subscribe')
                self.callbacks.append(receive)
                return object()

            def get_node_names_and_namespaces(self):
                owner.events.append('graph')
                owner.fail('graph')
                return [(name, '/')]

            def get_publishers_info_by_topic(self, topic):
                return []

            def get_subscriptions_info_by_topic(self, topic):
                return []

            def destroy_node(self):
                owner.events.append('destroy')
                self.destroyed = True
                owner.fail('destroy')

        node = Node()
        self.nodes.append(node)
        return node

    def spin(self, node, *, timeout_sec):
        self.events.append('spin')
        self.fail('spin')
        for callback in node.callbacks:
            callback(SimpleNamespace(timestamp=100_000_000, timestamp_sample=99_900_000))
        self.clock += 1.1

    def run(self, module=None):
        module = module or load_collector()
        # 真实函数执行，而非直接测试清理 helper；缺少发布接口的 Node 也约束其只读用途。
        with (
            patch.dict(sys.modules, self.modules),
            patch.object(module.time, 'monotonic', side_effect=lambda: self.clock),
            patch.object(module.time, 'time', return_value=100.5),
        ):
            return module.sample_ros(1.0)


class PassiveCollectorLifecycleTests(unittest.TestCase):
    def test_normal_report_and_observer_cleanup_order(self):
        fixture = CollectFixture()
        result = fixture.run()
        self.assertEqual(result['status'], 'ok')
        self.assertEqual(len(result['topics']), 9)
        self.assertTrue(all(row['messages'] == 1 for row in result['topics'].values()))
        self.assertEqual(fixture.events[-2:], ['destroy', 'shutdown'])
        self.assertFalse(fixture.active)

    def test_destroy_failure_still_attempts_context_shutdown(self):
        failure = OSError(errno.EIO, 'synthetic destroy failure')
        fixture = CollectFixture({'destroy': failure})
        with self.assertRaises(OSError) as raised:
            fixture.run()
        self.assertIs(raised.exception, failure)
        self.assertEqual(fixture.events[-2:], ['destroy', 'shutdown'])
        self.assertFalse(fixture.active)

    def test_shutdown_failure_remains_visible_after_node_destroy(self):
        failure = RuntimeError('synthetic shutdown failure')
        fixture = CollectFixture({'shutdown': failure})
        with self.assertRaises(RuntimeError) as raised:
            fixture.run()
        self.assertIs(raised.exception, failure)
        self.assertEqual(fixture.events[-2:], ['destroy', 'shutdown'])

    def test_spin_error_keeps_its_identity_and_cause_when_both_cleanups_fail(self):
        original = ValueError('original callback input')
        failure = RuntimeError('synthetic spin failure')
        failure.__cause__ = original
        destroy = OSError(errno.EIO, 'synthetic destroy failure')
        shutdown = OSError(errno.ENOSPC, 'synthetic shutdown failure')
        fixture = CollectFixture({'spin': failure, 'destroy': destroy, 'shutdown': shutdown})
        with self.assertRaises(RuntimeError) as raised:
            fixture.run()
        self.assertIs(raised.exception, failure)
        self.assertIs(failure.__cause__, original)
        self.assertEqual(visible_chain(failure), [failure, original, destroy, shutdown])
        self.assertEqual(fixture.events[-2:], ['destroy', 'shutdown'])

    def test_cleanup_error_preserves_its_existing_cause(self):
        original = ValueError('destroy original cause')
        destroy = RuntimeError('synthetic destroy failure')
        destroy.__cause__ = original
        shutdown = OSError(errno.EIO, 'synthetic shutdown failure')
        fixture = CollectFixture({'destroy': destroy, 'shutdown': shutdown})
        with self.assertRaises(RuntimeError) as raised:
            fixture.run()
        self.assertIs(raised.exception, destroy)
        self.assertIs(destroy.__cause__, original)
        self.assertEqual(visible_chain(destroy), [destroy, original, shutdown])

    def test_partial_init_error_closes_only_newly_activated_context(self):
        failure = RuntimeError('init failed after activation')
        fixture = CollectFixture({'init': failure}, partial_init=True)
        with self.assertRaises(RuntimeError) as raised:
            fixture.run()
        self.assertIs(raised.exception, failure)
        self.assertEqual(fixture.events, ['init', 'shutdown'])
        self.assertFalse(fixture.active)

    def test_partial_init_and_shutdown_errors_keep_init_first(self):
        failure = RuntimeError('init failed after activation')
        shutdown = OSError(errno.EIO, 'shutdown failure')
        fixture = CollectFixture({'init': failure, 'shutdown': shutdown}, partial_init=True)
        with self.assertRaises(RuntimeError) as raised:
            fixture.run()
        self.assertIs(raised.exception, failure)
        self.assertEqual(visible_chain(failure), [failure, shutdown])
        self.assertEqual(fixture.events, ['init', 'shutdown'])

    def test_init_error_before_activation_does_not_claim_context(self):
        failure = RuntimeError('init rejected before activation')
        fixture = CollectFixture({'init': failure})
        with self.assertRaises(RuntimeError) as raised:
            fixture.run()
        self.assertIs(raised.exception, failure)
        self.assertEqual(fixture.events, ['init'])
        self.assertFalse(fixture.active)

    def test_already_active_context_is_neither_adopted_nor_closed(self):
        fixture = CollectFixture(active=True)
        with self.assertRaisesRegex(RuntimeError, 'borrowed Context'):
            fixture.run()
        self.assertEqual(fixture.events, ['init'])
        self.assertTrue(fixture.active)

    def test_context_query_error_does_not_attempt_init_or_shutdown(self):
        failure = RuntimeError('context inspection failed')
        fixture = CollectFixture({'ok': failure})
        with self.assertRaises(RuntimeError) as raised:
            fixture.run()
        self.assertIs(raised.exception, failure)
        self.assertEqual(fixture.events, [])

    def test_node_construction_error_still_shuts_owned_context(self):
        failure = OSError(errno.ENOMEM, 'synthetic node allocation failure')
        fixture = CollectFixture({'create': failure})
        with self.assertRaises(OSError) as raised:
            fixture.run()
        self.assertIs(raised.exception, failure)
        self.assertEqual(fixture.events, ['init', 'create', 'shutdown'])
        self.assertFalse(fixture.active)

    def test_graph_error_keeps_primary_and_releases_resources(self):
        failure = RuntimeError('graph observation failure')
        fixture = CollectFixture({'graph': failure})
        with self.assertRaises(RuntimeError) as raised:
            fixture.run()
        self.assertIs(raised.exception, failure)
        self.assertEqual(fixture.events[-2:], ['destroy', 'shutdown'])
        self.assertFalse(fixture.active)

    def test_subscription_errors_keep_original_partial_report_contract(self):
        fixture = CollectFixture({'subscribe': OSError(errno.ENOMEM, 'no subscription')})
        result = fixture.run()
        self.assertTrue(all(row['status'] == 'unknown' for row in result['topics'].values()))
        self.assertEqual(fixture.events[-2:], ['destroy', 'shutdown'])

    def test_stop_exceptions_do_not_skip_later_cleanup(self):
        for kind in (KeyboardInterrupt, SystemExit):
            with self.subTest(kind=kind.__name__):
                primary, cleanup = kind('synthetic stop'), kind('synthetic cleanup stop')
                fixture = CollectFixture({'spin': primary, 'destroy': cleanup})
                with self.assertRaises(kind) as raised:
                    fixture.run()
                self.assertIs(raised.exception, primary)
                self.assertEqual(visible_chain(primary), [primary, cleanup])
                self.assertEqual(fixture.events[-2:], ['destroy', 'shutdown'])

    def test_repeated_start_stop_releases_each_owned_node_and_context(self):
        fixture = CollectFixture()
        for cycle in range(12):
            with self.subTest(cycle=cycle):
                self.assertEqual(fixture.run()['status'], 'ok')
                self.assertFalse(fixture.active)
                self.assertTrue(all(node.destroyed for node in fixture.nodes))
        self.assertEqual(len(fixture.nodes), 12)
        self.assertEqual(fixture.events.count('destroy'), 12)
        self.assertEqual(fixture.events.count('shutdown'), 12)


if __name__ == '__main__':
    unittest.main()
