# 探针清理故障注入：所有 ROS/Agent 均为替身，只写临时目录，不接触飞控或服务。
"""Exercise passive-probe finalization errors and default-context ownership."""
from contextlib import ExitStack
import errno
import importlib
import io
from pathlib import Path
import sys
import tempfile
import traceback
import types
import unittest
from unittest.mock import patch


class ProbeHarness:
    """以推进的合成接地反馈驱动真实 probe；事件记录用于检查清理先后及异常身份。"""

    def __init__(
        self,
        module,
        *,
        write_error=None,
        destroy_error=None,
        shutdown_error=None,
        spin_error=None,
        init_error=None,
        create_error=None,
        subscribe_error=None,
        initially_active=False,
        init_activates_before_error=False,
        initial_ok_error=None,
        boot_id_error=None,
    ):
        self.module = module
        self.write_error = write_error
        self.destroy_error = destroy_error
        self.shutdown_error = shutdown_error
        self.spin_error = spin_error
        self.init_error = init_error
        self.create_error = create_error
        self.subscribe_error = subscribe_error
        self.initially_active = initially_active
        self.init_activates_before_error = init_activates_before_error
        self.initial_ok_error = initial_ok_error
        self.boot_id_error = boot_id_error
        self.events = []
        self.active = initially_active
        self.clock = 1000.0
        self.nodes = []
        self.ok_calls = 0

    def __enter__(self):
        self.stack = ExitStack()
        self.folder = Path(self.stack.enter_context(tempfile.TemporaryDirectory()))
        self.evidence_path = self.folder / 'probe.json'
        harness = self

        class Node:
            def __init__(self):
                self.callbacks = []

            def create_subscription(self, cls, topic, callback, qos):
                harness.events.append('subscribe')
                if harness.subscribe_error is not None:
                    raise harness.subscribe_error
                self.callbacks.append(callback)
                return object()

            def count_publishers(self, topic):
                return 1

            def destroy_node(self):
                harness.events.append('destroy')
                if harness.destroy_error is not None:
                    raise harness.destroy_error

        def init():
            self.events.append('init')
            if self.init_error is not None:
                if self.init_activates_before_error:
                    self.active = True
                raise self.init_error
            if self.active:
                raise RuntimeError('borrowed context already initialized')
            self.active = True

        def create_node(name):
            self.events.append('create')
            if self.create_error is not None:
                raise self.create_error
            node = Node()
            self.nodes.append(node)
            return node

        def spin(node, timeout_sec):
            if self.spin_error is not None:
                raise self.spin_error
            self.clock += 0.5
            msg = types.SimpleNamespace(
                timestamp=int(self.clock * 1e6),
                system_id=1,
                component_id=1,
                arming_state=1,
                landed=True,
                at_rest=True,
            )
            for callback in node.callbacks:
                callback(msg)

        def ok():
            self.ok_calls += 1
            if self.ok_calls == 1 and self.initial_ok_error is not None:
                raise self.initial_ok_error
            return self.active

        def shutdown():
            self.events.append('shutdown')
            if self.shutdown_error is not None:
                raise self.shutdown_error
            self.active = False

        ros = types.ModuleType('rclpy')
        ros.init, ros.create_node, ros.spin_once = init, create_node, spin
        ros.ok, ros.shutdown = ok, shutdown
        qos = types.ModuleType('rclpy.qos')
        qos.qos_profile_sensor_data = object()
        msgs = types.ModuleType('px4_msgs.msg')
        msgs.VehicleStatus = msgs.VehicleLandDetected = object
        self.stack.enter_context(
            patch.dict(sys.modules, {'rclpy': ros, 'rclpy.qos': qos, 'px4_msgs.msg': msgs})
        )
        m = self.module
        for name, value in (
            ('process_list', []),
            ('uart_owners', set()),
            ('choose_agent', types.SimpleNamespace(pid=3215)),
        ):
            self.stack.enter_context(patch.object(m, name, return_value=value))
        self.stack.enter_context(
            patch.object(m.subprocess, 'Popen', side_effect=AssertionError('retain Agent'))
        )
        self.stack.enter_context(patch.object(m.time, 'time', side_effect=lambda: self.clock))
        self.stack.enter_context(
            patch.object(m.time, 'monotonic', side_effect=lambda: self.clock - 100)
        )
        original_write = Path.write_text
        original_read = Path.read_text

        def write(path, text, *args, **kwargs):
            if path == self.evidence_path:
                self.events.append('write')
                if self.write_error is not None:
                    raise self.write_error
            return original_write(path, text, *args, **kwargs)

        def read(path, *args, **kwargs):
            if path == Path('/proc/sys/kernel/random/boot_id'):
                if self.boot_id_error is not None:
                    raise self.boot_id_error
                return 'fixture-boot-id\n'
            return original_read(path, *args, **kwargs)

        self.stack.enter_context(patch.object(Path, 'write_text', autospec=True, side_effect=write))
        self.stack.enter_context(patch.object(Path, 'read_text', autospec=True, side_effect=read))
        self.stack.enter_context(patch('sys.stdout', new=io.StringIO()))
        return self

    def __exit__(self, *args):
        return self.stack.__exit__(*args)

    def run(self):
        return self.module.probe(
            self.folder,
            self.folder / 'not-a-real-agent',
            self.evidence_path,
            require_existing_agent=True,
        )


class ProbeLifecycleTests(unittest.TestCase):
    def module(self):
        return importlib.import_module('navigation_probe')

    def test_evidence_filesystem_errors_do_not_skip_node_or_context_cleanup(self):
        for code in (errno.ENOSPC, errno.EACCES, errno.EROFS, errno.EDQUOT):
            error = OSError(code, 'fixture evidence write')
            with self.subTest(errno=code), ProbeHarness(self.module(), write_error=error) as h:
                with self.assertRaises(OSError) as raised:
                    h.run()
                self.assertIs(raised.exception, error)
                self.assertEqual(h.events[-3:], ['write', 'destroy', 'shutdown'])
                self.assertFalse(h.active)

    def test_evidence_metadata_read_error_also_cleans_resources(self):
        error = OSError(errno.EIO, 'fixture boot id read')
        with ProbeHarness(self.module(), boot_id_error=error) as h:
            with self.assertRaises(OSError) as raised:
                h.run()
            self.assertIs(raised.exception, error)
            self.assertEqual(h.events[-2:], ['destroy', 'shutdown'])
            self.assertFalse(h.active)

    def test_real_directory_as_evidence_target_does_not_skip_cleanup(self):
        with ProbeHarness(self.module()) as h:
            h.evidence_path.mkdir()
            with self.assertRaises(IsADirectoryError) as raised:
                h.run()
            self.assertEqual(raised.exception.errno, errno.EISDIR)
            self.assertEqual(h.events[-3:], ['write', 'destroy', 'shutdown'])
            self.assertFalse(h.active)

    def test_real_invalid_parent_fails_before_claiming_ros_resources(self):
        with ProbeHarness(self.module(), initially_active=True) as h:
            parent = h.folder / 'a-regular-file'
            parent.write_text('not a directory')
            h.evidence_path = parent / 'probe.json'
            with self.assertRaises(FileExistsError):
                h.run()
            self.assertEqual(h.events, [])
            self.assertTrue(h.active)

    def test_destroy_failure_does_not_prevent_context_shutdown(self):
        error = RuntimeError('fixture destroy failure')
        with ProbeHarness(self.module(), destroy_error=error) as h:
            with self.assertRaises(RuntimeError) as raised:
                h.run()
            self.assertIs(raised.exception, error)
            self.assertEqual(h.events[-3:], ['write', 'destroy', 'shutdown'])
            self.assertFalse(h.active)

    def test_first_finalization_error_remains_primary_when_cleanup_also_fails(self):
        write = OSError(errno.ENOSPC, 'fixture disk full')
        destroy = RuntimeError('fixture destroy failed')
        shutdown = RuntimeError('fixture shutdown failed')
        with ProbeHarness(
            self.module(), write_error=write, destroy_error=destroy, shutdown_error=shutdown
        ) as h:
            with self.assertRaises(OSError) as raised:
                h.run()
            self.assertIs(raised.exception, write)
            rendered = ''.join(traceback.format_exception(type(write), write, write.__traceback__))
            self.assertIn(str(destroy), rendered)
            self.assertIn(str(shutdown), rendered)
            self.assertEqual(h.events[-3:], ['write', 'destroy', 'shutdown'])
            self.assertTrue(h.active, 'failed shutdown is not reported as successful release')

    def test_body_error_and_explicit_cause_survive_all_finalization_errors(self):
        cause = ValueError('fixture original cause')
        primary = RuntimeError('fixture callback failed')
        primary.__cause__ = cause
        write = OSError(errno.ENOSPC, 'fixture secondary disk full')
        with ProbeHarness(
            self.module(),
            spin_error=primary,
            write_error=write,
            destroy_error=RuntimeError('fixture secondary destroy'),
            shutdown_error=RuntimeError('fixture secondary shutdown'),
        ) as h:
            with self.assertRaises(RuntimeError) as raised:
                h.run()
            self.assertIs(raised.exception, primary)
            self.assertIs(primary.__cause__, cause)
            rendered = ''.join(
                traceback.format_exception(type(primary), primary, primary.__traceback__)
            )
            for text in (
                'fixture original cause',
                'fixture secondary disk full',
                'fixture secondary destroy',
                'fixture secondary shutdown',
            ):
                self.assertIn(text, rendered)
            self.assertEqual(h.events[-3:], ['write', 'destroy', 'shutdown'])

    def test_interruptions_propagate_after_cleanup(self):
        for error in (KeyboardInterrupt(), SystemExit(7), MemoryError('fixture allocation')):
            with (
                self.subTest(error=type(error).__name__),
                ProbeHarness(self.module(), spin_error=error) as h,
            ):
                with self.assertRaises(type(error)) as raised:
                    h.run()
                self.assertIs(raised.exception, error)
                self.assertEqual(h.events[-3:], ['write', 'destroy', 'shutdown'])
                self.assertFalse(h.active)

    def test_finalization_interrupt_or_allocation_failure_still_attempts_later_cleanup(self):
        for name, error in (
            ('write_error', MemoryError('fixture evidence allocation')),
            ('destroy_error', KeyboardInterrupt()),
            ('destroy_error', SystemExit(9)),
        ):
            with (
                self.subTest(stage=name, error=type(error).__name__),
                ProbeHarness(self.module(), **{name: error}) as h,
            ):
                with self.assertRaises(type(error)) as raised:
                    h.run()
                self.assertIs(raised.exception, error)
                self.assertEqual(h.events[-3:], ['write', 'destroy', 'shutdown'])
                self.assertFalse(h.active)

    def test_node_or_subscription_construction_failure_releases_owned_context(self):
        for name in ('create_error', 'subscribe_error'):
            error = RuntimeError('fixture ' + name)
            with self.subTest(stage=name), ProbeHarness(self.module(), **{name: error}) as h:
                with self.assertRaises(RuntimeError) as raised:
                    h.run()
                self.assertIs(raised.exception, error)
                self.assertEqual(h.events[-1], 'shutdown')
                self.assertEqual('destroy' in h.events, name == 'subscribe_error')
                self.assertFalse(h.active)

    def test_init_failure_cleans_only_newly_activated_context(self):
        for partial in (False, True):
            error = RuntimeError('fixture init failure')
            with (
                self.subTest(partial=partial),
                ProbeHarness(
                    self.module(), init_error=error, init_activates_before_error=partial
                ) as h,
            ):
                with self.assertRaises(RuntimeError) as raised:
                    h.run()
                self.assertIs(raised.exception, error)
                self.assertEqual('shutdown' in h.events, partial)
                self.assertFalse(h.active)

    def test_existing_context_is_not_shutdown_after_init_refusal(self):
        with ProbeHarness(self.module(), initially_active=True) as h:
            with self.assertRaisesRegex(RuntimeError, 'borrowed context'):
                h.run()
            self.assertNotIn('shutdown', h.events)
            self.assertTrue(h.active)

    def test_initial_context_query_failure_does_not_claim_ownership(self):
        error = RuntimeError('fixture initial ok failed')
        with ProbeHarness(self.module(), initial_ok_error=error, initially_active=True) as h:
            with self.assertRaises(RuntimeError) as raised:
                h.run()
            self.assertIs(raised.exception, error)
            self.assertNotIn('init', h.events)
            self.assertNotIn('shutdown', h.events)
            self.assertTrue(h.active)

    def test_repeated_success_and_filesystem_failure_leave_no_owned_context(self):
        with ProbeHarness(self.module()) as h:
            for turn in range(20):
                h.write_error = (
                    OSError(errno.ENOSPC, 'fixture repeated write') if turn % 2 else None
                )
                if h.write_error is None:
                    self.assertEqual(h.run(), 0)
                else:
                    with self.assertRaises(OSError) as raised:
                        h.run()
                    self.assertIs(raised.exception, h.write_error)
                self.assertFalse(h.active)
                self.assertEqual(h.events[-3:], ['write', 'destroy', 'shutdown'])
            self.assertEqual(h.events.count('init'), 20)
            self.assertEqual(h.events.count('destroy'), 20)
            self.assertEqual(h.events.count('shutdown'), 20)


if __name__ == '__main__':
    unittest.main()
