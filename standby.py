#!/usr/bin/env python3
"""Closed-gate background supervision; no mission or flight authorization."""
from __future__ import annotations

import argparse
import fcntl
import json
import logging
from logging.handlers import RotatingFileHandler
import math
import os
from pathlib import Path
import platform
import queue
import select
import shutil
import signal
import socket
import stat
import subprocess
import sys
import threading
import time

from fc01_control import readiness_issues, runtime_profile
from launcher import (Channel, ChannelError, ControlError, NodeStop, PROJECT, connect_controller,
                      finish_agent, make_socket_path, preflight, private_dir,
                      verify_binary)

RETRY_SECONDS = 30


def standby_config(base):
    config = runtime_profile(base, False)
    # A YAML quoted empty string is valid. A CLI zero-length RHS is not.
    config['fc01_system']['ros__parameters']['csv_path'] = ''
    return config


def rotating_logger(path, max_bytes=4*1024*1024):
    log = logging.Logger(str(path), level=logging.INFO)
    handler = BackgroundFileHandler(path, maxBytes=max_bytes, backupCount=2, encoding='utf8')
    handler.setFormatter(logging.Formatter('%(asctime)s %(message)s'))
    log.addHandler(handler)
    return log


def background_log_owner(log):
    # These handlers are closed by their daemon owner. logging.shutdown's
    # implicit main-thread lock/flush would otherwise defeat bounded shutdown.
    for reference in list(logging._handlerList):
        if reference() in log.handlers:
            logging._removeHandlerRef(reference)


class ConsoleOutput:
    """Best-effort worker output without Python's shared stdout buffer lock."""
    def __init__(self, output):
        self.fd = self.sock = None
        try:
            original = output.fileno()
            if stat.S_ISSOCK(os.fstat(original).st_mode):
                # MSG_DONTWAIT applies to this send; it does not change the
                # shared open-file flags of systemd's journal socket.
                self.sock = socket.socket(fileno=os.dup(original))
            else:
                # dup()+set_blocking() would alter the original stdout too.
                # /proc opens an independent file description for pipes/TTYs.
                self.fd = os.open(f'/proc/self/fd/{original}',
                                  os.O_WRONLY | os.O_NONBLOCK | os.O_CLOEXEC | os.O_NOCTTY)
        except (AttributeError, OSError, ValueError):
            pass  # The rotating log remains the full transition record.

    def write(self, message):
        data = (message[:1000] + ('…' if len(message) > 1000 else '') + '\n').encode('utf8', errors='replace')
        try:
            if self.sock is not None:
                self.sock.send(data, socket.MSG_DONTWAIT | socket.MSG_NOSIGNAL)
            elif self.fd is not None:
                os.write(self.fd, data)
        except OSError:
            pass  # Full/closed console sinks never hold the status worker.

    def close(self):
        if self.sock is not None:
            self.sock.close()
            self.sock = None
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None


class BackgroundFileHandler(RotatingFileHandler):
    """Report storage errors without acquiring Python's shared stderr lock."""
    def __init__(self, *args, **kwargs):
        self.write_errors = 0
        self.last_write_error = ''
        self.error_output = None
        super().__init__(*args, **kwargs)
        self.error_output = ConsoleOutput(sys.stderr)

    def record_error(self):
        self.write_errors += 1
        error = sys.exc_info()[1]
        self.last_write_error = f'{type(error).__name__}: {error}'[:512]
        if self.error_output is not None:
            self.error_output.write(f'Log storage failure ({self.write_errors}): {self.last_write_error}')

    def handleError(self, record):
        # The default handler prints a traceback to buffered sys.stderr. A
        # full pipe there can hold both this daemon and interpreter shutdown.
        self.record_error()

    def close(self):
        try:
            super().close()
        except (OSError, ValueError):
            # A failed stream can fail its final flush too. Keep that failure
            # out of threading.excepthook, which also uses buffered stderr.
            self.record_error()
        finally:
            if self.error_output is not None:
                self.error_output.close()
                self.error_output = None


class StatusWriter(threading.Thread):
    """One pending snapshot and a bounded transition log; no I/O in producers."""
    def __init__(self, folder, log):
        super().__init__(daemon=True)
        self.folder,self.log,self.output=folder,log,ConsoleOutput(sys.stdout)
        self.latest=queue.Queue(maxsize=1)
        self.messages=queue.Queue(maxsize=16)
        self.closing=threading.Event()
        background_log_owner(log)
        self.start()

    @staticmethod
    def replace(pending, value):
        try:pending.put_nowait(value)
        except queue.Full:
            try:pending.get_nowait()
            except queue.Empty:pass
            try:pending.put_nowait(value)
            except queue.Full:pass

    def submit(self, row):
        self.replace(self.latest,{**row,'updated_unix':time.time()})

    def message(self, text, *args):
        self.replace(self.messages,text % args if args else text)

    def close(self, timeout=.2):
        self.closing.set()
        self.join(timeout=timeout)

    def run(self):
        try:
            while not self.closing.is_set() or not self.latest.empty() or not self.messages.empty():
                try:row=self.latest.get(timeout=.05)
                except queue.Empty:row=None
                if row is not None:
                    row = {**row, 'status_log_write_errors':sum(
                        getattr(handler, 'write_errors', 0) for handler in self.log.handlers),
                        'status_log_last_error':next((handler.last_write_error for handler in self.log.handlers
                                                     if getattr(handler, 'last_write_error', '')), None)}
                    try:write_status(self.folder,row)
                    except (OSError,ValueError):pass  # Old snapshot keeps its old timestamp.
                for _ in range(16):
                    try:message=self.messages.get_nowait()
                    except queue.Empty:break
                    try:self.log.info('%s',message)
                    except (OSError,ValueError):pass
                    self.output.write(message)
        finally:
            self.output.close()
            for handler in self.log.handlers:handler.close()


class LogPump(threading.Thread):
    """Bounded reads, including output without newlines; no growing line buffer."""
    def __init__(self, stream, log):
        super().__init__(daemon=True)
        self.stream, self.log = stream, log
        background_log_owner(log)
        self.start()

    def run(self):
        try:
            while True:
                chunk = os.read(self.stream.fileno(), 4096)
                if not chunk:
                    return
                self.log.info('%s', chunk.decode('utf8', errors='replace').rstrip('\n'))
        finally:
            self.stream.close()
            for handler in self.log.handlers:handler.close()


def write_status(folder, payload):
    content = {'flight_actions_enabled': False, 'rc_navigation_enabled': False,
               'updated_unix': time.time(), **payload}
    temp = folder/'status.json.tmp'
    with open(temp, 'w', encoding='utf8') as f:
        json.dump(content, f, ensure_ascii=False, allow_nan=False)
        f.write('\n')
    os.replace(temp, folder/'status.json')


def controller_result(node):
    code = node.poll()
    number = -code if code is not None and code < 0 else None
    try:
        name = signal.Signals(number).name if number is not None else None
    except ValueError:
        name = 'UNKNOWN'
    reason = None
    if code is not None:
        reason = ('原生控制器退出，code='+str(code) if number is None else
                  f'原生控制器被信号终止，signal={number} ({name})，returncode={code}')
    return dict(controller_returncode=code,
                controller_exit_code=code if code is not None and code >= 0 else None,
                controller_exit_signal=number, controller_exit_signal_name=name,
                controller_exit_reason=reason)


def ipc_error(exc):
    if isinstance(exc, ChannelError):
        return exc.as_dict()
    result = dict(kind='socket_error', message=str(exc))
    if isinstance(exc, OSError) and exc.errno is not None:
        result['errno'] = exc.errno
    return result


def monitor(channel, node, stopped, observe, health=lambda: None, *, diagnostics=None):
    """Only STATUS; retain the process until its own graceful exit completes."""
    state, state_at, failure = {}, -math.inf, None
    diagnostics = {} if diagnostics is None else diagnostics
    stop = NodeStop(node)
    last_emit, previous = -math.inf, None
    def receive():
        nonlocal state, state_at
        for obj in channel.read():
            if obj.get('kind') in ('state', 'reply') and 'phase' in obj:
                state, state_at = obj, time.monotonic()
        if channel.read_error is not None:
            raise channel.read_error

    def row(status, reason, current=False):
        now = time.monotonic()
        return dict(diagnostics, status=status, reason=reason, state=state,
                    state_age_seconds=max(0, now-state_at) if state_at != -math.inf else None,
                    feedback_current=current, readiness_issues=readiness_issues(state),
                    controller_pid=node.pid, state_controller_pid=node.pid if state else None)
    try:
        channel.send('STATUS', [])
    except (ControlError, OSError) as exc:
        failure = str(exc)
        diagnostics['ipc_error'] = ipc_error(exc)
    while node.poll() is None:
        now = time.monotonic()
        if failure is None and not stopped.is_set():
            failure = health()
        closing = stopped.is_set() or failure is not None
        if closing:
            stop.request()
        if channel is not None:
            try:
                if select.select([channel.sock], [], [], .05)[0]:
                    receive()
            except (ControlError, OSError) as exc:
                diagnostics['ipc_error'] = ipc_error(exc)
                if failure is None and not stopped.is_set():
                    failure = str(exc)
                channel.sock.close()
                channel = None
        else:
            stopped.wait(.05) if not closing else time.sleep(.05)
        now = time.monotonic()
        closing = stopped.is_set() or failure is not None
        if closing:
            stop.request()
        fresh = (channel is not None and 0 <= now-state_at <= 3 and state.get('status_fresh') is True
                 and state.get('local_fresh') is True)
        status = 'STOPPING' if closing else 'CONNECTED' if fresh else 'WAITING'
        reason = failure or ('等待原生退出' if closing else
                            '已连接；后台动作门关闭' if fresh else '等待飞控新鲜状态和本地位置')
        key = (status, reason, state.get('phase'), state.get('reason'))
        if key != previous or now-last_emit >= 1:
            observe(row(status, reason, fresh))
            previous, last_emit = key, now
    if channel is not None:
        try:
            receive()
        except (ControlError, OSError) as exc:
            diagnostics['ipc_error'] = ipc_error(exc)
            if failure is None and not stopped.is_set():
                failure = str(exc)
        channel.sock.close()
    diagnostics.update(controller_result(node))
    if not stopped.is_set() and failure is None:
        failure = diagnostics['controller_exit_reason']
    observe(row('STOPPING' if stopped.is_set() else 'WAITING',
                failure or diagnostics['controller_exit_reason'] or '等待原生退出'))
    return state, state_at, failure


def validate(root, agent, *, controller=None):
    """Validate SDK/configuration; pre-build preparation has no controller yet."""
    if platform.machine().lower() not in ('aarch64', 'arm64'):
        raise ControlError('仅供 ARM64 香橙派安装/运行')
    if controller is not None:
        verify_binary(controller)
    import yaml
    with open(root/'config/attitude_altitude.yaml', encoding='utf8') as f:
        config = standby_config(yaml.safe_load(f))
    if not agent.is_file() or not os.access(agent, os.X_OK):
        raise ControlError('缺少离线 Agent: '+str(agent))
    if shutil.which('fuser') is None:
        raise ControlError('缺少 fuser；未安装服务')
    import px4_msgs.msg as msgs
    from rclpy.type_support import check_for_type_support
    for name in ('BatteryStatus', 'VehicleAttitude', 'VehicleLocalPosition',
                 'VehicleStatus', 'VehicleLandDetected', 'FailsafeFlags'):
        check_for_type_support(getattr(msgs, name))
    return config


def supervise(root, agent_binary, folder, stopped, *, controller=None, config_factory=None,
              status_metadata=None, row_formatter=None):
    import yaml
    private_dir(folder)
    cache = Path.home()/'.cache/fc01_operator'
    private_dir(cache)
    log = rotating_logger(folder/'standby.log', max_bytes=1024*1024)
    writer = StatusWriter(folder,log)
    last_message = None
    last_state, last_state_at, last_state_pid = {}, -math.inf, None
    diagnostics = {}
    metadata = dict(status_metadata or {})
    metadata['supervisor_pid'] = os.getpid()
    try:
        metadata['boot_id'] = Path('/proc/sys/kernel/random/boot_id').read_text().strip()
    except OSError:
        metadata['boot_id'] = ''

    def publish(row):
        nonlocal last_message, last_state, last_state_at, last_state_pid
        row = {**diagnostics, **row}
        now = time.monotonic()
        if row.get('state'):
            last_state = row['state']
            age = row.get('state_age_seconds')
            last_state_at = now-age if type(age) in (int, float) and math.isfinite(age) else -math.inf
            last_state_pid = row.get('state_controller_pid', row.get('controller_pid'))
        else:
            row.update(state=last_state, feedback_current=False, state_controller_pid=last_state_pid,
                       state_age_seconds=max(0, now-last_state_at) if last_state_at != -math.inf else None)
        row = row_formatter(row) if row_formatter else row
        row = {**metadata, **row}
        writer.submit(row)
        key = row['status']+': '+row['reason']
        if key != last_message:
            writer.message(key)
            last_message = key

    try:
        while not stopped.is_set():
            node = agent = lock = pump = channel = ipc = native_log = None
            state, state_at, existing = {}, -math.inf, None
            failure = None
            try:
                lockfd = os.open(cache/'launcher.lock', os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW, 0o600)
                lock = os.fdopen(lockfd, 'w')
                try:
                    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
                except BlockingIOError as exc:
                    raise ControlError('旧入口正在运行；保留它，等待其退出') from exc
                binary, existing = preflight(root, agent_binary, '/dev/ttyS2',
                                             **({'controller':controller} if controller is not None else {}))
                config = config_factory(root, agent_binary) if config_factory else validate(root, agent_binary, controller=binary)
                with open(folder/'runtime.yaml', 'w', encoding='utf8') as f:
                    yaml.safe_dump(config, f, allow_unicode=True, sort_keys=False)
                ipc = make_socket_path(cache/'ipc')
                if existing is None:
                    # An Agent retained after shutdown has no pipe dependency on us.
                    agent = subprocess.Popen([str(agent_binary), 'serial', '-D', '/dev/ttyS2',
                                              '-b', '115200', '-v', '0'],
                                             stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
                                             stderr=subprocess.DEVNULL, start_new_session=True)
                # Open storage before the controller starts; the pump owns
                # this attempt's handlers after it starts reading stdout.
                native_log = rotating_logger(folder/'controller.log')
                node = subprocess.Popen([str(binary), '--ros-args', '--disable-external-lib-logs',
                                         '--params-file', str(folder/'runtime.yaml'),
                                         '-p', 'socket_path:='+str(ipc)],
                                        stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                                        stderr=subprocess.STDOUT, start_new_session=True)
                # Keep the last native outcome through failed reconnect checks;
                # a successfully started replacement begins a new IPC lifetime.
                diagnostics = {}
                # Each pump solely owns its handlers. An old pump delayed by
                # storage cannot close a replacement controller's logger.
                pump = LogPump(node.stdout, native_log)
                publish(dict(status='WAITING', reason='控制器已启动，等待飞控反馈', controller_pid=node.pid,
                             agent_pid=agent.pid if agent else existing.pid, agent_owned=agent is not None))
                channel = connect_controller(ipc, node)
                def observe(row):
                    row.update(agent_pid=agent.pid if agent else existing.pid, agent_owned=agent is not None)
                    publish(row)
                next_health_check = 0
                def health():
                    nonlocal next_health_check
                    if time.monotonic() < next_health_check:
                        return None
                    next_health_check = time.monotonic()+1
                    if agent is not None:
                        return '自建 Agent 已退出' if agent.poll() is not None else None
                    try:
                        argv = tuple(x.decode(errors='replace') for x in
                                     Path('/proc', str(existing.pid), 'cmdline').read_bytes().split(b'\0') if x)
                        return None if argv == existing.argv else '复用的 Agent 已退出或进程已变化'
                    except OSError:
                        return '复用的 Agent 已退出'
                state, state_at, failure = monitor(channel, node, stopped, observe, health,
                                                  diagnostics=diagnostics)
                if state:
                    last_state, last_state_at, last_state_pid = state, state_at, node.pid
                channel = None
            except Exception as exc:
                failure = str(exc)[:2000]
            finally:
                if node is not None and node.poll() is None:
                    NodeStop(node).request()
                    publish(dict(status='STOPPING', reason='等待原生控制器优雅退出', controller_pid=node.pid))
                    while node.poll() is None:
                        time.sleep(.1)
                if channel is not None:
                    channel.sock.close()
                if node is not None:
                    diagnostics.update(controller_result(node))
                if pump is not None:
                    pump.join(timeout=3)
                elif native_log is not None:
                    for handler in native_log.handlers:handler.close()
                if ipc is not None:
                    try:
                        ipc.unlink(missing_ok=True)
                        ipc.parent.rmdir()
                    except OSError as exc:
                        writer.message('保留本次 IPC 目录：%s', exc)
                if agent is not None:
                    if finish_agent(agent, True, node is not None and node.poll() == 0,
                                    state, state_at, time.monotonic()):
                        try:
                            agent.wait(timeout=5)
                        except subprocess.TimeoutExpired:
                            writer.message('Agent 尚未退出，未强制终止 PID=%s', agent.pid)
                    else:
                        writer.message('保留自建 Agent PID=%s；缺少新鲜未解锁接地退出证据', agent.pid)
                if lock is not None:
                    lock.close()
            if not stopped.is_set():
                remaining = RETRY_SECONDS
                while remaining > 0 and not stopped.is_set():
                    reason = failure or '等待重新连接'
                    exit_reason = diagnostics.get('controller_exit_reason')
                    if exit_reason and exit_reason not in reason:
                        reason += '；'+exit_reason
                    publish(dict(status='WAITING', reason=reason, retry_in_seconds=remaining))
                    step = min(5, remaining)
                    if stopped.wait(step):
                        break
                    remaining -= step
        publish(dict(status='STOPPED', reason='后台待命已停止'))
    finally:
        writer.close()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--validate', action='store_true')
    parser.add_argument('--root', type=Path, default=Path(os.environ.get('FC01_PROJECT_ROOT', str(Path.home()/PROJECT))))
    parser.add_argument('--agent', type=Path, default=Path.home()/'fc01_agent_build_offline/MicroXRCEAgent')
    args = parser.parse_args(argv)
    os.umask(0o077)
    root, agent = args.root.expanduser().resolve(), args.agent.expanduser().resolve()
    if args.validate:
        validate(root, agent, controller=root/'install_r632/jikl_fc01_system/lib/jikl_fc01_system/fc01_system_node')
        print('ROS / px4_msgs / ARM64 控制器 / 关闭动作门配置检查通过。')
        return 0
    stopped = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, lambda *_: stopped.set())
    supervise(root, agent, Path.home()/'FC01_autostart', stopped)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
