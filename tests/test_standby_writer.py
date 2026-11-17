# 异步状态/日志输出的阻塞与退出回归：磁盘、终端、journal 都不能拥有 IPC 或恢复进程生命周期。
# 部分 fixture 在子进程里故意填满真实管道或 /dev/full；父进程有超时和 finally 清理。
"""Slow status storage must never own the native process/IPC lifecycle."""
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

import yaml
import launcher
import navigation
import standby

ROOT = Path(__file__).resolve().parents[1]


# 分别隔离观测时间、快照合并、stdout/stderr、日志 handler 和 supervisor 的线程边界。
class StandbyWriterTests(unittest.TestCase):
    # 延后落盘必须保留 submit 时观测戳，写盘时间不能刷新旧飞机反馈。
    def test_delayed_snapshot_keeps_observation_time_and_renders_stale(self):
        entered = threading.Event()
        release = threading.Event()
        real_write = standby.write_status
        with tempfile.TemporaryDirectory() as d:
            folder = Path(d)
            log = standby.rotating_logger(folder / 'writer.log')

            def delayed_write(*args, **kwargs):
                entered.set()
                release.wait(2)
                real_write(*args, **kwargs)

            with (
                patch.object(standby, 'write_status', side_effect=delayed_write),
                patch.object(standby.time, 'time', return_value=1000),
            ):
                writer = standby.StatusWriter(folder, log)
                writer.submit(
                    dict(
                        status='HEIGHT_HOLD',
                        reason='hold',
                        supervisor_pid=10,
                        feedback_current=True,
                        state_age_seconds=0.1,
                        state=dict(status_fresh=True, local_fresh=True),
                    )
                )
                self.assertTrue(entered.wait(1))
            release.set()
            writer.close(timeout=1)
            row = json.loads((folder / 'status.json').read_text())
            self.assertEqual(
                row['updated_unix'], 1000, 'writing late must not refresh old aircraft feedback'
            )
            self.assertIn(
                '[STALE]',
                navigation.render_status(row, dict(ActiveState='active', MainPID='10'), now=1020),
            )

    # 输出线程被堵时 close 有界；1000 个待写快照只保留 sequence=999。
    def test_blocked_status_has_bounded_close_and_only_latest_pending_snapshot(self):
        entered = threading.Event()
        release = threading.Event()
        with tempfile.TemporaryDirectory() as d:
            folder = Path(d)
            log = standby.rotating_logger(folder / 'writer.log')
            real_write = standby.write_status

            def block_write(*args, **kwargs):
                entered.set()
                release.wait(3)
                real_write(*args, **kwargs)

            with patch.object(standby, 'write_status', side_effect=block_write):
                writer = standby.StatusWriter(folder, log)
                writer.submit(dict(status='WAITING', sequence=-1))
                self.assertTrue(entered.wait(1))
            try:
                for sequence in range(1000):
                    writer.submit(dict(status='WAITING', sequence=sequence))
                begin = time.monotonic()
                writer.close()
                self.assertLess(
                    time.monotonic() - begin,
                    0.5,
                    'shutdown cannot join a blocked terminal sink indefinitely',
                )
                release.set()
                writer.join(timeout=1)
                self.assertFalse(writer.is_alive())
                row = json.loads((folder / 'status.json').read_text())
                self.assertEqual(
                    row['sequence'], 999, 'the newest pending state must replace obsolete snapshots'
                )
            finally:
                release.set()
                writer.join(timeout=1)

    # 填满真实 stdout 且父端不读，状态仍落盘、进程仍退出，不能修改共享 FD 的阻塞标志。
    def test_full_real_stdout_pipe_cannot_block_status_or_python_exit(self):
        with tempfile.TemporaryDirectory() as d:
            code = '''import fcntl,json,os,pathlib,standby,sys,threading,time
folder=pathlib.Path(sys.argv[1])
capacity=fcntl.fcntl(1,fcntl.F_GETPIPE_SZ)
os.write(1,b'x'*capacity)
original_flags=fcntl.fcntl(1,fcntl.F_GETFL)
log=standby.rotating_logger(folder/'writer.log')
recorded=threading.Event()
real_emit=log.handlers[0].emit
def emit(record):
    real_emit(record);recorded.set()
log.handlers[0].emit=emit
writer=standby.StatusWriter(folder,log)
writer.message('terminal reader stopped')
assert recorded.wait(1)
time.sleep(.1)
writer.submit(dict(status='WAITING',sequence=999))
writer.close()
assert fcntl.fcntl(1,fcntl.F_GETFL)==original_flags
assert json.loads((folder/'status.json').read_text())['sequence']==999
'''
            child = subprocess.Popen(
                [sys.executable, '-c', code, d],
                cwd=ROOT,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
            )
            try:
                # Deliberately do not drain stdout until the child has exited.
                child.wait(timeout=3)
                self.assertEqual(child.returncode, 0, child.stderr.read().decode(errors='replace'))
            finally:
                if child.poll() is None:
                    child.kill()
                child.communicate(timeout=3)

    # 以 socketpair 模拟 journal 满缓冲，验证非阻塞尝试不改 stdout 原 flag。
    def test_full_journal_socket_does_not_change_stdout_flags_or_block_exit(self):
        with tempfile.TemporaryDirectory() as d:
            code = '''import fcntl,json,os,pathlib,socket,standby,sys
folder=pathlib.Path(sys.argv[1])
original_flags=fcntl.fcntl(1,fcntl.F_GETFL)
journal=socket.socket(fileno=os.dup(1))
while True:
    try:journal.send(b'x'*4096,socket.MSG_DONTWAIT)
    except BlockingIOError:break
journal.close()
writer=standby.StatusWriter(folder,standby.rotating_logger(folder/'writer.log'))
writer.message('journal reader stopped')
writer.submit(dict(status='WAITING',sequence=999))
writer.close(timeout=1)
assert not writer.is_alive()
assert fcntl.fcntl(1,fcntl.F_GETFL)==original_flags
assert json.loads((folder/'status.json').read_text())['sequence']==999
'''
            reader, output = socket.socketpair()
            child = subprocess.Popen(
                [sys.executable, '-c', code, d], cwd=ROOT, stdout=output, stderr=subprocess.PIPE
            )
            output.close()
            try:
                child.wait(timeout=3)
                self.assertEqual(child.returncode, 0, child.stderr.read().decode(errors='replace'))
            finally:
                if child.poll() is None:
                    child.kill()
                child.communicate(timeout=3)
                reader.close()

    # /dev/full 拒绝日志且 stderr 也满时，不得递归阻塞异常日志；状态保留错误计数。
    def test_disk_full_and_full_stderr_pipe_preserve_status_and_bounded_exit(self):
        with tempfile.TemporaryDirectory() as d:
            code = '''import fcntl,json,logging,os,pathlib,standby,sys,threading
folder=pathlib.Path(sys.argv[1])
os.write(2,b'x'*fcntl.fcntl(2,fcntl.F_GETPIPE_SZ))
original_flags=fcntl.fcntl(2,fcntl.F_GETFL)
original_error_policy=logging.raiseExceptions
log=standby.rotating_logger(folder/'writer.log')
handler=log.handlers[0]
handler.stream.close();handler.stream=open('/dev/full','w');handler.maxBytes=0
reported=threading.Event()
real_error=handler.handleError
def handle_error(record):
    real_error(record);reported.set()
handler.handleError=handle_error
writer=standby.StatusWriter(folder,log)
writer.message('disk full while the terminal reader stopped')
assert reported.wait(1)
writer.submit(dict(status='WAITING',sequence=999))
writer.close(timeout=1)
assert not writer.is_alive()
assert handler.write_errors>=1
assert fcntl.fcntl(2,fcntl.F_GETFL)==original_flags
assert logging.raiseExceptions==original_error_policy
row=json.loads((folder/'status.json').read_text())
assert row['sequence']==999 and row['status_log_write_errors']>=1
'''
            child = subprocess.Popen(
                [sys.executable, '-c', code, d],
                cwd=ROOT,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE,
            )
            try:
                # The error pipe remains full while /dev/full rejects log writes.
                child.wait(timeout=3)
                self.assertEqual(child.returncode, 0, child.stderr.read().decode(errors='replace'))
            finally:
                if child.poll() is None:
                    child.kill()
                child.communicate(timeout=3)

    # 后台 handler 永久等待不能让解释器退出跟着无界 join。
    def test_blocked_background_handler_cannot_hold_python_exit(self):
        with tempfile.TemporaryDirectory() as d:
            code = '''import pathlib,threading,standby,sys
folder=pathlib.Path(sys.argv[1])
log=standby.rotating_logger(folder/'writer.log')
entered=threading.Event();release=threading.Event()
def blocked(record):
    entered.set();release.wait(30)
log.handlers[0].emit=blocked
writer=standby.StatusWriter(folder,log)
writer.message('blocked disk')
assert entered.wait(1)
writer.close()
'''
            result = subprocess.run(
                [sys.executable, '-c', code, d], cwd=ROOT, capture_output=True, text=True, timeout=3
            )
            self.assertEqual(result.returncode, 0, result.stderr)

    # 状态写线程冻结时，主监督循环仍应读完真实 peer 并优雅终止，trace 只含 STATUS。
    def test_blocked_status_write_cannot_prevent_ipc_read_or_graceful_stop(self):
        entered = threading.Event()
        release = threading.Event()
        stopped = threading.Event()
        children = []
        errors = []
        writers = []
        real_popen = subprocess.Popen
        real_read = Path.read_bytes
        real_writer = standby.StatusWriter

        # 保存实际 writer 引用用于最终释放/等待，防止测试留下后台线程。
        def make_writer(*args, **kwargs):
            writer = real_writer(*args, **kwargs)
            writers.append(writer)
            return writer

        with tempfile.TemporaryDirectory() as d:
            home = Path(d)
            binary = home / 'native_fixture'
            binary.write_text((ROOT / 'tests/standby_native.py').read_text())
            binary.chmod(0o700)
            base = yaml.safe_load((ROOT / 'tests/fixtures/base.yaml').read_text())
            a, b = socket.socketpair()
            existing = launcher.Process(os.getpid(), os.getuid(), ('fixture-agent',))

            def block_write(*_args, **_kwargs):
                entered.set()
                release.wait(4)

            def read_bytes(path):
                if path == Path('/proc', str(existing.pid), 'cmdline'):
                    return b'fixture-agent\0'
                return real_read(path)

            def spawn(argv, **kwargs):
                child = real_popen(
                    [*argv, '--test-fd=' + str(b.fileno())], pass_fds=(b.fileno(),), **kwargs
                )
                children.append(child)
                return child

            def run():
                try:
                    standby.supervise(home / 'project', home / 'agent', home / 'state', stopped)
                except BaseException as exc:
                    errors.append(exc)

            worker = threading.Thread(target=run, daemon=True)
            try:
                with (
                    patch.object(Path, 'home', return_value=home),
                    patch.object(standby, 'preflight', return_value=(binary, existing)),
                    patch.object(standby, 'validate', return_value=standby.standby_config(base)),
                    patch.object(standby, 'write_status', side_effect=block_write),
                    patch.object(standby, 'StatusWriter', side_effect=make_writer),
                    patch.object(standby.subprocess, 'Popen', side_effect=spawn),
                    patch.object(standby, 'connect_controller', return_value=launcher.Channel(a)),
                    patch.object(Path, 'read_bytes', new=read_bytes),
                ):
                    worker.start()
                    self.assertTrue(entered.wait(1), 'status writer never ran')
                    time.sleep(0.12)
                    stopped.set()
                    worker.join(1)
                    self.assertFalse(
                        worker.is_alive(), 'blocked status storage prevented native graceful stop'
                    )
                    self.assertFalse(errors, errors)
                    trace = json.loads((home / 'state/native_trace.json').read_text())
                    self.assertEqual(
                        trace['ops'],
                        ['STATUS'],
                        'monitor must keep reading real IPC while status storage stalls',
                    )
                    self.assertEqual(children[0].returncode, 0)
            finally:
                release.set()
                stopped.set()
                worker.join(3)
                for writer in writers:
                    writer.join(timeout=1)
                for child in children:
                    if child.poll() is None:
                        child.terminate()
                    child.wait(timeout=3)
                a.close()
                b.close()


if __name__ == '__main__':
    unittest.main()
