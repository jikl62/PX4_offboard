# 控制入口的线协议、资源所有权和优雅停止回归：使用真实 socketpair/临时文件及无害子进程。
# 所有断言都围绕“不把旧反馈、未知进程或错误回复变成控制许可”，不访问真实串口。
import importlib
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import tempfile
import time
import unittest
import zipfile

ROOT = Path(__file__).resolve().parents[1]


# 加载当前 launcher；入口缺失时直接报错，而不是跳过关键回归。
def api():
    if not (ROOT / 'launcher.py').is_file():
        raise AssertionError('launcher is not implemented yet')
    return importlib.import_module('launcher')


# 检验 JSON 行分帧与终止错误分类；一条合法最终状态不能因其后的坏尾部丢失。
class ProtocolTests(unittest.TestCase):
    # 分别注入干净 EOF、截断 JSON、非法 UTF-8、NaN 和超长行，必须保留不同错误类别。
    def test_terminal_reads_classify_clean_eof_truncation_and_invalid_messages(self):
        m = api()
        cases = ((b'', 'clean_eof'), (b'{"kind":', 'truncated_json'),
                 (b'[]\n', 'invalid_message'), (b'{broken}\n', 'invalid_message'),
                 (b'{"x":NaN}\n', 'invalid_message'),
                 (b'{"x":"\xff"}\n', 'invalid_message'),
                 (b'x' * 131073, 'invalid_message'))
        for wire, kind in cases:
            with self.subTest(kind=kind, size=len(wire)):
                a, b = socket.socketpair()
                try:
                    channel = m.Channel(a)
                    b.sendall(wire)
                    b.close()
                    with self.assertRaises(m.ControlError) as caught:
                        channel.read()
                    self.assertEqual(getattr(caught.exception, 'kind', None), kind)
                finally:
                    a.close()
                    b.close()

    # 先交付完整 SAFE_DISARMED，下一次 read 再抛尾部错误，防止丢掉最后安全证据。
    def test_complete_messages_survive_truncated_or_invalid_tail_before_error(self):
        m = api()
        for tail, kind in ((b'{"kind":', 'truncated_json'),
                           (b'[]\n', 'invalid_message')):
            with self.subTest(kind=kind):
                a, b = socket.socketpair()
                try:
                    channel = m.Channel(a)
                    b.sendall(b'{"kind":"state","phase":"SAFE_DISARMED","armed":false}\n'+tail)
                    b.close()
                    try:
                        rows = channel.read()
                    except m.ControlError as exc:
                        self.fail('complete state was lost before terminal error: '+str(exc))
                    self.assertEqual(rows, [dict(kind='state', phase='SAFE_DISARMED', armed=False)])
                    with self.assertRaises(m.ControlError) as caught:
                        channel.read()
                    self.assertEqual(getattr(caught.exception, 'kind', None), kind)
                finally:
                    a.close()
                    b.close()

    # 主动关闭本地 FD 是 socket_error，并需保留 errno；不能误报对端正常 EOF。
    def test_socket_failure_is_distinct_from_peer_eof(self):
        m = api()
        a, b = socket.socketpair()
        self.addCleanup(b.close)
        channel = m.Channel(a)
        a.close()
        try:
            channel.read()
        except Exception as exc:
            self.assertIsInstance(exc, m.ControlError)
            self.assertEqual(getattr(exc, 'kind', None), 'socket_error')
            self.assertIsInstance(getattr(exc, 'errno', None), int)
        else:
            self.fail('closed local socket was accepted')

    # 命令序号/白名单/有限参数限制共同阻止换行注入及 bool 冒充数值。
    def test_wire_commands_are_bounded_finite_and_not_injectable(self):
        m = api()
        self.assertEqual(m.encode_command(7, 'TAKEOFF', [0.5]), b'7 TAKEOFF 0.5\n')
        for seq, op, args in ((0, 'PING', []), (True, 'PING', []),
                              (1, 'LAND\n2 TAKEOFF', []),
                              (1, 'TAKEOFF', [float('nan')]),
                              (1, 'TAKEOFF', [True]), (1, 'ARM_ONLY', [])):
            with self.subTest(op=op, args=args):
                with self.assertRaises(m.ControlError):
                    m.encode_command(seq, op, args)

    # 通过 socketpair 把 reply 拆成两次发送，再紧邻 state，验证缓存按完整换行交付。
    def test_channel_assembles_real_fragmented_socket_replies(self):
        m = api()
        a, b = socket.socketpair()
        self.addCleanup(a.close)
        self.addCleanup(b.close)
        channel = m.Channel(a)
        self.assertEqual(channel.send('PING', []), 1)
        self.assertEqual(b.recv(128), b'1 PING\n')
        b.sendall(b'{"kind":"reply","seq":1,')
        self.assertEqual(channel.read(), [])
        b.sendall(b'"accepted":true}\n{"kind":"state","phase":"IDLE"}\n')
        rows = channel.read()
        self.assertEqual([r['kind'] for r in rows], ['reply', 'state'])
        self.assertIs(rows[0]['accepted'], True)

    # 坏 JSON、非对象、非有限值和超过上限的未终止行必须中止编排。
    def test_malformed_or_unbounded_messages_close_the_plan(self):
        m = api()
        for wire in (b'[]\n', b'{broken}\n', b'{"x":NaN}\n', b'x' * 131073):
            with self.subTest(size=len(wire)):
                a, b = socket.socketpair()
                try:
                    ch = m.Channel(a)
                    b.sendall(wire)
                    with self.assertRaises(m.ControlError):
                        ch.read()
                finally:
                    a.close()
                    b.close()

    # 最后一条完整上锁反馈先交付，后续 read 才报告断流。
    def test_final_complete_state_is_delivered_before_eof(self):
        m = api()
        a, b = socket.socketpair()
        self.addCleanup(a.close)
        ch = m.Channel(a)
        b.sendall(b'{"kind":"state","phase":"SAFE_DISARMED","armed":false}\n')
        b.close()
        rows = ch.read()
        self.assertEqual(rows[-1]['phase'], 'SAFE_DISARMED')
        with self.assertRaises(m.ControlError):
            ch.read()


# 检验 Agent 的 UID/波特率/设备/占用者一致性，以及私有 socket 路径保护。
class OwnershipTests(unittest.TestCase):
    # 生成仅用于比较的 Process 描述，不启动该 Agent；默认契约为 ttyS2、115200。
    def proc(self, pid, baud='115200', device='/dev/ttyS2'):
        m = api()
        return m.Process(pid, os.getuid(), ('/home/orangepi/fc01_agent_build_offline/MicroXRCEAgent',
                                           'serial', '-D', device, '-b', baud, '-v', '4'))

    # 只能复用被当前设备占用证据确认的唯一匹配 Agent，未知占用者使启动失败。
    def test_reuses_exact_agent_and_rejects_unknown_uart_owner(self):
        m = api()
        p = self.proc(123)
        self.assertEqual(m.choose_agent([p], {123}, '/dev/ttyS2', os.getuid()), p)
        self.assertIsNone(m.choose_agent([], set(), '/dev/ttyS2', os.getuid()))
        with self.assertRaises(m.ControlError):
            m.choose_agent([p], {999}, '/dev/ttyS2', os.getuid())

    # 错误波特率、其他用户或多 Agent 竞争时，不得任选一个继续。
    def test_wrong_baud_other_uid_and_duplicate_agents_are_not_reused(self):
        m = api()
        bad_uid = m.Process(123, os.getuid()+1, self.proc(123).argv)
        for procs, owners in (([self.proc(123, '921600')], {123}),
                              ([self.proc(123), self.proc(124)], {123, 124}),
                              ([bad_uid], {123})):
            with self.subTest(procs=procs):
                with self.assertRaises(m.ControlError):
                    m.choose_agent(procs, owners, '/dev/ttyS2', os.getuid())

    # 构造 AArch64 ELF 标记检查，拒绝旧版本或 x86 架构，校验过程不篡改文件。
    def test_incompatible_binary_is_refused_without_patching_it(self):
        m = api()
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / 'node'
            header = bytearray(64)
            header[:6] = b'\x7fELF\x02\x01'
            header[18:20] = (183).to_bytes(2, 'little')
            payload = bytes(header) + b' '.join(m.BINARY_MARKERS)
            p.write_bytes(payload)
            p.chmod(0o700)
            m.verify_binary(p)
            self.assertEqual(p.read_bytes(), payload)
            p.write_bytes(payload.replace(m.BINARY_MARKERS[0], b'old'))
            with self.assertRaises(m.ControlError):
                m.verify_binary(p)
            p.write_bytes(payload[:18] + (62).to_bytes(2, 'little') + payload[20:])
            with self.assertRaises(m.ControlError):
                m.verify_binary(p)

    # 每次会话分配新 0700 目录；已有普通文件及 symlink 均不能被删除或借道写入。
    def test_private_socket_paths_preserve_previous_sessions(self):
        m = api()
        with tempfile.TemporaryDirectory() as d:
            root = Path(d) / 'ipc'
            one = m.make_socket_path(root)
            one.write_text('sentinel')
            two = m.make_socket_path(root)
            self.assertNotEqual(one, two)
            self.assertEqual(one.read_text(), 'sentinel')
            self.assertEqual(two.parent.stat().st_mode & 0o777, 0o700)
            bad = Path(d) / 'bad'
            bad.symlink_to(root, target_is_directory=True)
            with self.assertRaises(m.ControlError):
                m.make_socket_path(bad)


# 检验首故障证据、白名单报告和退出时的安全资源释放顺序。
class ReportAndLifecycleTests(unittest.TestCase):
    # 后续健康状态不抹首故障；无关敏感文件与指向它的日志链接不得进 ZIP。
    def test_report_keeps_first_failure_and_only_named_session_artifacts(self):
        m = api()
        with tempfile.TemporaryDirectory() as d:
            e = m.Evidence(Path(d) / 'run', {'mode': 'check', 'domain': '0'})
            self.addCleanup(e.events.close)
            e.record({'kind': 'reply', 'seq': 1, 'accepted': False, 'message': 'FIRST'}, 1.0)
            e.record({'kind': 'state', 'phase': 'SAFETY', 'reason': 'SECOND'}, 2.0)
            e.record({'kind': 'state', 'phase': 'IDLE', 'armed': False}, 3.0)
            (e.folder / 'private.env').write_text('TOKEN=should-not-be-exported')
            (e.folder / 'agent.log').symlink_to(e.folder / 'private.env')
            report = e.report('finished')
            with zipfile.ZipFile(report) as z:
                self.assertNotIn('private.env', z.namelist())
                self.assertNotIn('agent.log', z.namelist())
                summary = json.loads(z.read('summary.json'))
                self.assertEqual(summary['first_failure']['message'], 'FIRST')
                self.assertEqual(summary['last_state']['phase'], 'IDLE')
                self.assertNotIn('TOKEN', ''.join(z.namelist()))

    # 只有自己创建的 Agent、原生已退出及新鲜接地上锁反馈同时成立才允许清理。
    def test_owned_agent_stops_only_after_fresh_disarmed_landed_node_exit(self):
        m = api()
        p = subprocess.Popen([sys.executable, '-c', 'import time;time.sleep(30)'])
        self.addCleanup(lambda: p.poll() is None and p.terminate())
        state = {'phase': 'SAFE_DISARMED', 'armed': False, 'landed': True, 'status_fresh': True, 'land_fresh': True}
        self.assertFalse(m.finish_agent(p, False, True, state, 10.0, 10.1))
        self.assertIsNone(p.poll())
        self.assertFalse(m.finish_agent(p, True, False, state, 10.0, 10.1))
        self.assertFalse(m.finish_agent(p, True, True, state, 1.0, 10.1))
        self.assertFalse(m.finish_agent(p, True, True, {**state, 'armed': True}, 10.0, 10.1))
        self.assertFalse(m.finish_agent(p, True, True, {**state, 'phase': 'REQUEST_ARM'}, 10.0, 10.1))
        self.assertIsNone(p.poll())
        self.assertTrue(m.finish_agent(p, True, True, state, 10.0, 10.1))
        p.wait(timeout=3)
        self.assertIsNotNone(p.returncode)

    # 重复 request 只发送一次 SIGTERM；信号被收到后仍需等待原生自行完成恢复退出。
    def test_controller_stop_is_graceful_single_signal_not_a_kill(self):
        m = api()
        with tempfile.TemporaryDirectory() as d:
            proof = Path(d) / 'stop'
            ready = Path(d) / 'ready'
            script = ('import signal,time,pathlib;'
                      f'p=pathlib.Path({str(proof)!r});'
                      'signal.signal(signal.SIGTERM,lambda *_:p.write_text("TERM"));'
                      f'pathlib.Path({str(ready)!r}).write_text("ready");'
                      'time.sleep(2)')
            p = subprocess.Popen([sys.executable, '-c', script])
            self.addCleanup(lambda: p.poll() is None and p.terminate())
            end = time.monotonic()+2
            while not ready.exists() and time.monotonic() < end:
                time.sleep(0.01)
            owner = m.NodeStop(p)
            owner.request()
            owner.request()
            end = time.monotonic()+1
            while not proof.exists() and time.monotonic() < end:
                time.sleep(0.01)
            self.assertEqual(proof.read_text(), 'TERM')
            self.assertIsNone(p.poll(), 'shutdown must wait for the native recovery loop')
            p.wait(timeout=3)


if __name__ == '__main__':
    unittest.main()
