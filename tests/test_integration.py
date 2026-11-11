# 真实 Unix socket/子进程的端到端生命周期测试；测试 peer 模拟飞控，不连接硬件。
# 核心合同是退出/终端阻塞时仍保持恢复 PING，直到原生侧反馈接地上锁才关闭与生成报告。
import contextlib
import importlib
import io
import json
import os
from pathlib import Path
import socket
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]


# 把任务编排、IPC 读写、终端输出和原生退出放到同一进程边界验证。
class LifecycleIntegration(unittest.TestCase):
    @staticmethod
    def console_ground_observer(module):
        # 以 Mission 真正消费了地面反馈作为输入握手；进程启动 0.1 秒不代表已收到状态。
        ground_seen = threading.Event()
        original_observe = module.Mission.observe

        def observe(mission, state, now):
            # 原方法先更新任务状态，测试只观察结果，不伪造反馈或放宽任何准备条件。
            original_observe(mission, state, now)
            if state.get('phase') == 'IDLE' and not module.readiness_issues(state):
                # 只有完整、未解锁的可准备地面状态，才允许模拟操作员发送起飞输入。
                ground_seen.set()

        return ground_seen, observe

    # 共用恢复场景：等待真实模拟地面反馈、只发一次起飞，再制造退出或输出阻塞。
    def run_quit_with_output_or_report(
        self, output=None, slow_report=False, pending_arm=False, end_event='quit'
    ):
        m = importlib.import_module('launcher')
        ground_seen, observe = self.console_ground_observer(m)
        with tempfile.TemporaryDirectory(prefix='fc01_io_') as d:
            root = Path(d)
            a, b = socket.socketpair()
            self.addCleanup(a.close)
            trace = root / 'trace.jsonl'
            command = [
                sys.executable,
                str(ROOT / 'tests/native_fixture.py'),
                str(b.fileno()),
                str(trace),
            ]
            if pending_arm:
                command.append('pending_arm')
            node = subprocess.Popen(command, pass_fds=(b.fileno(),))
            b.close()
            self.addCleanup(lambda: node.poll() is None and node.terminate())
            e = m.Evidence(root / 'session', {'mode': 'flight', 'test_only': True})
            self.addCleanup(e.events.close)
            rfd, wfd = os.pipe()
            reader = os.fdopen(rfd, 'r')
            self.addCleanup(reader.close)
            calls_while_native_alive = []
            original_report = e.report
            stop_signal = [False]

            def report(outcome):
                calls_while_native_alive.append(node.poll() is None)
                if slow_report:
                    time.sleep(3.2)  # Longer than the preserved 3 s console lease.
                return original_report(outcome)

            def operator():
                with os.fdopen(wfd, 'w', buffering=1) as writer:
                    # 最多等 3 秒真实反馈；超时退出，由主线程明确报告握手失败。
                    if not ground_seen.wait(timeout=3):
                        writer.write('quit\n')
                        return
                    writer.write('takeoff 0.5\n')
                    end = time.monotonic() + 3
                    while time.monotonic() < end:
                        if trace.exists() and 'TAKEOFF' in trace.read_text():
                            if end_event == 'quit':
                                writer.write('quit\n')
                            elif end_event == 'signal':
                                stop_signal[0] = True
                                while node.poll() is None:
                                    time.sleep(0.02)
                            return
                        time.sleep(0.02)
                    writer.write('quit\n')

            t = threading.Thread(target=operator)
            t.start()
            try:
                with (
                    patch.object(m.Mission, 'observe', observe),
                    patch.object(sys, 'stdin', reader),
                    patch.object(e, 'report', side_effect=report),
                    contextlib.redirect_stdout(output or io.StringIO()),
                ):
                    result = m.run_console(
                        m.Channel(a), node, e, {'aa_max_height': 5.0}, True, 15, stop_signal
                    )
                node.wait(timeout=3)
                report('after_native_exit')
                rows = [json.loads(row) for row in trace.read_text().splitlines()]
                ops = [r['op'] for r in rows]
                self.assertTrue(
                    ground_seen.is_set(), 'console did not consume ready ground feedback'
                )
                self.assertEqual(
                    result,
                    {'quit': 'operator_quit', 'eof': 'operator_eof', 'signal': 'operator_signal'}[
                        end_event
                    ],
                )
                self.assertEqual(ops.count('TAKEOFF'), 1)
                self.assertTrue(any(r['op'] == 'PING' for r in rows[ops.index('LAND') + 1 :]))
                self.assertFalse(
                    any(calls_while_native_alive), 'ZIP work must wait until native exit'
                )
                self.assertFalse(e.last_state['armed'])
            finally:
                t.join(timeout=4)
                if hasattr(output, 'release'):
                    output.release.set()

    # 报告故意阻塞超过控制台租约；ZIP 工作必须等原生退出后开始。
    def test_slow_report_cannot_interrupt_live_recovery_heartbeat(self):
        self.run_quit_with_output_or_report(slow_report=True)

    # 请求解锁尚未完成时分别退出、EOF、信号；迟到的 armed 反馈仍需被恢复链处理。
    def test_quit_eof_and_signal_during_pending_arm_keep_recovery_channel(self):
        for event in ('quit', 'eof', 'signal'):
            with self.subTest(event=event):
                self.run_quit_with_output_or_report(pending_arm=True, end_event=event)

    # BrokenPipeError 只能失去显示，不能放弃仍存活的原生恢复进程。
    def test_broken_stdout_cannot_abandon_native_recovery(self):
        class BrokenOutput:
            def write(self, text):
                raise BrokenPipeError('terminal disconnected')

            def flush(self):
                raise BrokenPipeError('terminal disconnected')

        self.run_quit_with_output_or_report(output=BrokenOutput())

    # 输出端冻结 3.2 秒时，IPC 心跳必须独立继续，不能等待终端恢复。
    def test_blocked_stdout_cannot_block_recovery_heartbeat(self):
        class BlockedOutput:
            def __init__(self):
                self.release = threading.Event()

            def write(self, text):
                self.release.wait(timeout=3.2)

            def flush(self):
                pass

        self.run_quit_with_output_or_report(output=BlockedOutput())

    # 只读检查模式的实际线协议仅允许 INTERACTIVE/STATUS/PING。
    def test_check_mode_never_sends_preparation_or_motion_commands(self):
        m = importlib.import_module('launcher')
        with tempfile.TemporaryDirectory(prefix='fc01_ck_') as d:
            root = Path(d)
            a, b = socket.socketpair()
            self.addCleanup(a.close)
            trace = root / 'trace.jsonl'
            node = subprocess.Popen(
                [
                    sys.executable,
                    str(ROOT / 'tests/native_fixture.py'),
                    str(b.fileno()),
                    str(trace),
                ],
                pass_fds=(b.fileno(),),
            )
            b.close()
            self.addCleanup(lambda: node.poll() is None and node.terminate())
            e = m.Evidence(root / 'session', {'mode': 'check', 'test_only': True})
            self.addCleanup(e.events.close)
            with contextlib.redirect_stdout(io.StringIO()):
                result = m.run_console(
                    m.Channel(a), node, e, {'aa_max_height': 5.0}, False, 0.3, [False]
                )
            node.wait(timeout=3)
            ops = [json.loads(row)['op'] for row in trace.read_text().splitlines()]
            self.assertTrue(set(ops).issubset({'INTERACTIVE', 'STATUS', 'PING'}))
            self.assertEqual(result, 'check_pass')

    # 检查 AUTHORITY_RC→ATTITUDE_READY→TAKEOFF→LAND 的顺序及 LAND 后 PING，最终必须 SAFE_DISARMED。
    def test_explicit_takeoff_quit_keeps_heartbeat_through_native_descent(self):
        m = importlib.import_module('launcher')
        ground_seen, observe = self.console_ground_observer(m)
        with tempfile.TemporaryDirectory(prefix='fc01_it_') as d:
            root = Path(d)
            trace = root / 'trace.jsonl'
            a, b = socket.socketpair()
            self.addCleanup(a.close)
            node = subprocess.Popen(
                [
                    sys.executable,
                    str(ROOT / 'tests/native_fixture.py'),
                    str(b.fileno()),
                    str(trace),
                ],
                pass_fds=(b.fileno(),),
            )
            b.close()
            self.addCleanup(lambda: node.poll() is None and node.terminate())
            channel = m.Channel(a)
            e = m.Evidence(root / 'session', {'mode': 'flight', 'test_only': True})
            self.addCleanup(e.events.close)
            rfd, wfd = os.pipe()
            reader = os.fdopen(rfd, 'r')
            self.addCleanup(reader.close)

            def operator():
                with os.fdopen(wfd, 'w', buffering=1) as writer:
                    # 此测试同样等到 Mission 已消费地面反馈，不依赖测试机的进程启动速度。
                    if not ground_seen.wait(timeout=3):
                        writer.write('quit\n')
                        return
                    writer.write('takeoff 0.5\n')
                    end = time.monotonic() + 3
                    while time.monotonic() < end:
                        if trace.exists() and 'TAKEOFF' in trace.read_text():
                            writer.write('quit\n')
                            return
                        time.sleep(0.02)
                    writer.write('quit\n')

            t = threading.Thread(target=operator)
            t.start()
            with (
                patch.object(m.Mission, 'observe', observe),
                patch.object(sys, 'stdin', reader),
                contextlib.redirect_stdout(io.StringIO()),
            ):
                result = m.run_console(channel, node, e, {'aa_max_height': 5.0}, True, 15, [False])
            t.join(timeout=4)
            node.wait(timeout=3)
            rows = [json.loads(line) for line in trace.read_text().splitlines()]
            ops = [row['op'] for row in rows]
            self.assertTrue(ground_seen.is_set(), 'console did not consume ready ground feedback')
            self.assertEqual(result, 'operator_quit')
            self.assertEqual(ops.count('TAKEOFF'), 1)
            self.assertLess(ops.index('AUTHORITY_RC'), ops.index('ATTITUDE_READY'))
            self.assertLess(ops.index('ATTITUDE_READY'), ops.index('TAKEOFF'))
            self.assertLess(ops.index('TAKEOFF'), ops.index('LAND'))
            after_land = rows[ops.index('LAND') + 1 :]
            self.assertTrue(
                any(r['op'] == 'PING' for r in after_land), 'lease must survive operator quit'
            )
            self.assertFalse(e.last_state['armed'])
            self.assertEqual(e.last_state['phase'], 'SAFE_DISARMED')


if __name__ == '__main__':
    unittest.main()
