#!/usr/bin/env python3
# 后台待命监督器：管理唯一原生控制器、串口 Agent、状态快照与有界日志。
# 默认配置关闭动作门；导航服务可提供配置工厂，但本监督器本身不发送起飞、解锁或任务命令。
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
from launcher import (
    Channel,
    ChannelError,
    ControlError,
    NodeStop,
    PROJECT,
    connect_controller,
    finish_agent,
    make_socket_path,
    preflight,
    private_dir,
    verify_binary,
    choose_agent,
    process_list,
    uart_owners,
)

RETRY_SECONDS = 30


# 传输恢复只处理 Agent，绝不替换仍在运行的原生控制器；后者保留原有安全状态并自行判断恢复。
class AgentTransport:
    """Recover UART transport only; never command or replace the live controller.

    The launcher lock remains held by supervise. A separate UART lock protects
    cooperating transport workers, and every restart checks actual UART owners.
    The worker stays alive during a requested graceful native shutdown because
    that shutdown may need fresh DDS evidence before it can safely finish.
    """

    def __init__(self, binary, owned, existing, cache, message):
        self.binary, self.owned, self.existing = binary, owned, existing
        self.cache, self.message = cache, message
        self.closing = threading.Event()
        # guard 保护对外信息快照；launch_guard 串行化“允许新启动”与 close，防止关停后又生成 Agent。
        self.guard = threading.Lock()
        self.launch_guard = threading.Lock()
        self.worker = None
        self.next_check, self.backoff, self.restarts = 0.0, 1.0, 0
        self.live_since = time.monotonic()
        self.info = {}
        self._report(
            'CONNECTED' if owned is not None or existing is not None else 'RECOVERING', None
        )

    # 只在状态/原因变化时写转换日志；界面取得信息副本，不直接读写传输线程的内部状态。
    def _report(self, status, reason):
        pid = (
            self.owned.pid
            if self.owned is not None
            else self.existing.pid if self.existing else None
        )
        info = dict(
            agent_pid=pid,
            agent_owned=self.owned is not None,
            agent_transport_status=status,
            agent_transport_reason=reason,
            agent_restart_count=self.restarts,
        )
        with self.guard:
            previous = (
                self.info.get('agent_transport_status'),
                self.info.get('agent_transport_reason'),
            )
            self.info = info
        if previous != (status, reason):
            self.message(
                'Agent transport %s: %s; PID=%s', status, reason or 'UART /dev/ttyS2 115200', pid
            )

    def snapshot(self):
        with self.guard:
            return dict(self.info)

    # 自建进程查 poll；复用进程重新核对 /proc 完整参数，防止仅凭旧 PID 误认已被复用的进程。
    def alive(self):
        if self.owned is not None:
            return self.owned.poll() is None
        if self.existing is None:
            return False
        try:
            argv = tuple(
                x.decode(errors='replace')
                for x in Path('/proc', str(self.existing.pid), 'cmdline').read_bytes().split(b'\0')
                if x
            )
            return argv == self.existing.argv
        except OSError:
            return False

    # 每轮先确认存活；失联后锁住 UART 启动路径并重新检查实际占用者。失败按 1 至 30 秒指数退避。
    def step(self, now=None):
        now = time.monotonic() if now is None else now
        if now < self.next_check or self.closing.is_set():
            return
        if self.alive():
            if now - self.live_since >= 10:
                self.backoff = 1.0
            self.next_check = now + 1
            return
        old_code = self.owned.poll() if self.owned is not None else None
        self.owned = self.existing = None
        self._report('RECOVERING', 'Agent unavailable; native retained, exit=' + str(old_code))
        try:
            fd = os.open(self.cache / 'agent.lock', os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW, 0o600)
            with os.fdopen(fd, 'w') as lock:
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
                existing = choose_agent(
                    process_list(), uart_owners('/dev/ttyS2'), '/dev/ttyS2', os.getuid()
                )
                # 占用检查完成后仍须在启动锁内再查 closing，关闭请求与 Popen 之间不能留下竞态窗口。
                with self.launch_guard:
                    if self.closing.is_set():
                        return
                    if existing is not None:
                        self.existing = existing
                    else:
                        if not self.binary.is_file() or not os.access(self.binary, os.X_OK):
                            raise ControlError('缺少可执行 Agent: ' + str(self.binary))
                        self.owned = subprocess.Popen(
                            [
                                str(self.binary),
                                'serial',
                                '-D',
                                '/dev/ttyS2',
                                '-b',
                                '115200',
                                '-v',
                                '0',
                            ],
                            stdin=subprocess.DEVNULL,
                            stdout=subprocess.DEVNULL,
                            stderr=subprocess.DEVNULL,
                            start_new_session=True,
                        )
                        self.restarts += 1
            self.live_since = now
            self.next_check = now + self.backoff
            self.backoff = min(30.0, self.backoff * 2)
            self._report('CONNECTED', 'transport restored; controller safety state retained')
        except (OSError, ControlError) as exc:
            self._report('RECOVERING', str(exc)[:1000])
            self.next_check = now + self.backoff
            self.backoff = min(30.0, self.backoff * 2)

    # 守护线程只定期恢复传输；异常转为诊断与 30 秒后重试，不向控制器发送任何飞行动作。
    def start(self):
        def run():
            while not self.closing.is_set():
                try:
                    self.step()
                except Exception as exc:
                    self._report('RECOVERING', 'transport check failed: ' + str(exc)[:1000])
                    self.next_check = time.monotonic() + 30
                self.closing.wait(0.2)

        self.worker = threading.Thread(target=run, daemon=True)
        self.worker.start()

    # 先设关闭事件，再等待已获准启动结束；关闭后返回自建 Agent 交给上层核对地面证据，不能直接杀掉。
    def close(self):
        self.closing.set()
        with self.launch_guard:
            pass  # Finish a launch already admitted before the stop request.
        if self.worker is not None:
            self.worker.join(timeout=5)
        # An in-flight ownership check cannot spawn after closing was set.
        return self.owned


# 若发现上次遗留的原生进程，保留它并只维护 Agent，等待其自身安全退出后再考虑新控制器。
def wait_for_previous_controller(agent_binary, cache, stopped, publish, message):
    """Keep transport available while an orphan completes native safe shutdown."""

    def controllers():
        return [p.pid for p in process_list() if Path(p.argv[0]).name == 'fc01_system_node']

    previous = controllers()
    if not previous:
        return not stopped.is_set()
    transport = AgentTransport(agent_binary, None, None, cache, message)
    transport.start()
    try:
        while previous and not stopped.is_set():
            publish(
                dict(
                    status='WAITING',
                    reason='等待已有控制器安全退出；仅恢复 Agent 传输',
                    previous_controller_pids=previous,
                    feedback_current=False,
                    **transport.snapshot(),
                )
            )
            if stopped.wait(1):
                break
            previous = controllers()
    finally:
        retained = transport.close()
        if retained is not None:
            message('保留恢复的 Agent PID=%s；已有控制器的安全退出由原生程序确认', retained.pid)
    return not stopped.is_set()


# 基于原配置生成关闭动作门的独立配置，空 csv_path 禁用同步 CSV；不会覆写传入的原始配置。
def standby_config(base):
    config = runtime_profile(base, False)
    # A YAML quoted empty string is valid. A CLI zero-length RHS is not.
    config['fc01_system']['ros__parameters']['csv_path'] = ''
    return config


# 每个日志拥有独立滚动处理器，限制磁盘增长；原生每次启动使用自己的处理器实例。
def rotating_logger(path, max_bytes=4 * 1024 * 1024):
    log = logging.Logger(str(path), level=logging.INFO)
    handler = BackgroundFileHandler(path, maxBytes=max_bytes, backupCount=2, encoding='utf8')
    handler.setFormatter(logging.Formatter('%(asctime)s %(message)s'))
    log.addHandler(handler)
    return log


# 处理器由后台线程关闭；从 logging 全局退出清单移除，避免主线程等待卡住的后台 flush 锁。
def background_log_owner(log):
    # These handlers are closed by their daemon owner. logging.shutdown's
    # implicit main-thread lock/flush would otherwise defeat bounded shutdown.
    for reference in list(logging._handlerList):
        if reference() in log.handlers:
            logging._removeHandlerRef(reference)


# 控制台只做尽力输出，不获得 Python 共享 stdout 锁；完整转换记录由滚动日志保存。
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
                self.fd = os.open(
                    f'/proc/self/fd/{original}',
                    os.O_WRONLY | os.O_NONBLOCK | os.O_CLOEXEC | os.O_NOCTTY,
                )
        except (AttributeError, OSError, ValueError):
            pass  # The rotating log remains the full transition record.

    # 单条控制台输出限制 1000 字符；满管道/断开只丢显示副本，不能阻塞状态工作线程。
    def write(self, message):
        data = (message[:1000] + ('…' if len(message) > 1000 else '') + '\n').encode(
            'utf8', errors='replace'
        )
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


# 记录磁盘写错误的次数与最后原因；错误输出也使用非阻塞通道，避免报错路径再次卡住。
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
            self.error_output.write(
                f'Log storage failure ({self.write_errors}): {self.last_write_error}'
            )

    # 覆写 logging 默认 traceback 路径：该路径可能锁住缓冲 stderr，从而拖住解释器退出。
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


# 状态只保留最新一份，转换消息最多排队 16 条；生产者只入队，不承担磁盘或终端 I/O。
class StatusWriter(threading.Thread):
    """One pending snapshot and a bounded transition log; no I/O in producers."""

    def __init__(self, folder, log):
        super().__init__(daemon=True)
        self.folder, self.log, self.output = folder, log, ConsoleOutput(sys.stdout)
        self.latest = queue.Queue(maxsize=1)
        self.messages = queue.Queue(maxsize=16)
        self.closing = threading.Event()
        background_log_owner(log)
        self.start()

    # 队列满时丢旧取新；与消费者并发取走时允许丢一次副本，绝不等待队列空位。
    @staticmethod
    def replace(pending, value):
        try:
            pending.put_nowait(value)
        except queue.Full:
            try:
                pending.get_nowait()
            except queue.Empty:
                pass
            try:
                pending.put_nowait(value)
            except queue.Full:
                pass

    # updated_unix 是此快照提交时间；飞控自身的新鲜度仍由 payload 中的状态和样本年龄说明。
    def submit(self, row):
        self.replace(self.latest, {**row, 'updated_unix': time.time()})

    def message(self, text, *args):
        self.replace(self.messages, text % args if args else text)

    def close(self, timeout=0.2):
        self.closing.set()
        self.join(timeout=timeout)

    # 状态写失败保留磁盘旧文件及旧时间戳；成功新快照中附带已观测到的日志存储错误。
    def run(self):
        try:
            while not self.closing.is_set() or not self.latest.empty() or not self.messages.empty():
                try:
                    row = self.latest.get(timeout=0.05)
                except queue.Empty:
                    row = None
                if row is not None:
                    row = {
                        **row,
                        'status_log_write_errors': sum(
                            getattr(handler, 'write_errors', 0) for handler in self.log.handlers
                        ),
                        'status_log_last_error': next(
                            (
                                handler.last_write_error
                                for handler in self.log.handlers
                                if getattr(handler, 'last_write_error', '')
                            ),
                            None,
                        ),
                    }
                    try:
                        write_status(self.folder, row)
                    except (OSError, ValueError):
                        pass  # Old snapshot keeps its old timestamp.
                for _ in range(16):
                    try:
                        message = self.messages.get_nowait()
                    except queue.Empty:
                        break
                    try:
                        self.log.info('%s', message)
                    except (OSError, ValueError):
                        pass
                    self.output.write(message)
        finally:
            self.output.close()
            for handler in self.log.handlers:
                handler.close()


# 管道 read 与日志行无关，先拼行再加时间戳；超长或无换行输出分片并持续排空，避免堵住原生 stdout。
class LogPump(threading.Thread):
    """Preserve native records across pipe reads, with bounded buffering.

    Pipe reads are unrelated to JSON record boundaries. Logging each 4096-byte
    read inserted timestamps inside state records and could glue startup onto
    an older partial record. Oversized/no-newline output is still drained.
    """

    def __init__(self, stream, log):
        super().__init__(daemon=True)
        self.stream, self.log = stream, log
        background_log_owner(log)
        self.start()

    def run(self):
        pending = b''
        limits = [
            handler.maxBytes // 2
            for handler in self.log.handlers
            if getattr(handler, 'maxBytes', 0) > 0
        ]
        # 单条重组上限不超过 64 KiB，并参考滚动文件的一半容量；异常输出不能无限占内存。
        record_limit = max(1024, min([65536, *limits]))
        try:
            while True:
                chunk = os.read(self.stream.fileno(), 4096)
                if not chunk:
                    if pending:
                        self.log.info('%s', pending.decode('utf8', errors='replace'))
                    return
                pending += chunk
                while pending:
                    newline = pending.find(b'\n')
                    if 0 <= newline <= record_limit:
                        record, pending = pending[:newline], pending[newline + 1 :]
                        self.log.info('%s', record.rstrip(b'\r').decode('utf8', errors='replace'))
                    elif len(pending) > record_limit:
                        record, pending = pending[:record_limit], pending[record_limit:]
                        self.log.info(
                            'NATIVE_LOG_RECORD_FRAGMENT %s', record.decode('utf8', errors='replace')
                        )
                    else:
                        break
        finally:
            self.stream.close()
            for handler in self.log.handlers:
                handler.close()


# 先写同目录临时文件再原子替换，读端得到完整 JSON；默认动作门值可被明确提供的 payload 覆盖。
def write_status(folder, payload):
    content = {
        'flight_actions_enabled': False,
        'rc_navigation_enabled': False,
        'updated_unix': time.time(),
        **payload,
    }
    temp = folder / 'status.json.tmp'
    with open(temp, 'w', encoding='utf8') as f:
        json.dump(content, f, ensure_ascii=False, allow_nan=False)
        f.write('\n')
    os.replace(temp, folder / 'status.json')


# 区分正常退出码与信号退出；只报告已发生的进程结果，不据此推断飞机是否接地或上锁。
def controller_result(node):
    code = node.poll()
    number = -code if code is not None and code < 0 else None
    try:
        name = signal.Signals(number).name if number is not None else None
    except ValueError:
        name = 'UNKNOWN'
    reason = None
    if code is not None:
        reason = (
            '原生控制器退出，code=' + str(code)
            if number is None
            else f'原生控制器被信号终止，signal={number} ({name})，returncode={code}'
        )
    return dict(
        controller_returncode=code,
        controller_exit_code=code if code is not None and code >= 0 else None,
        controller_exit_signal=number,
        controller_exit_signal_name=name,
        controller_exit_reason=reason,
    )


# 将连接终止原因保存成结构化诊断，保留 ChannelError 细分类型和系统 errno。
def ipc_error(exc):
    if isinstance(exc, ChannelError):
        return exc.as_dict()
    result = dict(kind='socket_error', message=str(exc))
    if isinstance(exc, OSError) and exc.errno is not None:
        result['errno'] = exc.errno
    return result


# 只发送一次 STATUS 并持续读状态，不进入 INTERACTIVE/PING 租约模式；请求停止后等原生自行结束。
def monitor(channel, node, stopped, observe, health=lambda: None, *, diagnostics=None):
    """Only STATUS; retain the process until its own graceful exit completes."""
    state, state_at, failure = {}, -math.inf, None
    diagnostics = {} if diagnostics is None else diagnostics
    stop = NodeStop(node)
    last_emit, previous = -math.inf, None

    # 每条带 phase 的回复/状态更新接收时间；已收的有效末尾状态先保留，再暴露锁存的 IPC 错误。
    def receive():
        nonlocal state, state_at
        for obj in channel.read():
            if obj.get('kind') in ('state', 'reply') and 'phase' in obj:
                state, state_at = obj, time.monotonic()
        if channel.read_error is not None:
            raise channel.read_error

    def row(status, reason, current=False):
        now = time.monotonic()
        return dict(
            diagnostics,
            status=status,
            reason=reason,
            state=state,
            state_age_seconds=max(0, now - state_at) if state_at != -math.inf else None,
            feedback_current=current,
            readiness_issues=readiness_issues(state),
            controller_pid=node.pid,
            state_controller_pid=node.pid if state else None,
        )

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
                if select.select([channel.sock], [], [], 0.05)[0]:
                    receive()
            except (ControlError, OSError) as exc:
                diagnostics['ipc_error'] = ipc_error(exc)
                if failure is None and not stopped.is_set():
                    failure = str(exc)
                channel.sock.close()
                channel = None
        else:
            stopped.wait(0.05) if not closing else time.sleep(0.05)
        now = time.monotonic()
        closing = stopped.is_set() or failure is not None
        if closing:
            stop.request()
        # 反馈当前有效要求连接仍在、接收未超过 3 秒且飞控 status/local 均新鲜；历史缓存不能满足此条件。
        fresh = (
            channel is not None
            and 0 <= now - state_at <= 3
            and state.get('status_fresh') is True
            and state.get('local_fresh') is True
        )
        status = 'STOPPING' if closing else 'CONNECTED' if fresh else 'WAITING'
        reason = failure or (
            '等待原生退出'
            if closing
            else '已连接；后台动作门关闭' if fresh else '等待飞控新鲜状态和本地位置'
        )
        key = (status, reason, state.get('phase'), state.get('reason'))
        if key != previous or now - last_emit >= 1:
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
    observe(
        row(
            'STOPPING' if stopped.is_set() else 'WAITING',
            failure or diagnostics['controller_exit_reason'] or '等待原生退出',
        )
    )
    return state, state_at, failure


# 安装/启动前检查 SDK、Agent 与类型支持；可接受安装包自带配置路径，避免旧 SDK 配置覆盖新导航策略。
def validate(root, agent, *, controller=None, config_path=None):
    """Validate SDK/configuration; pre-build preparation has no controller yet."""
    if platform.machine().lower() not in ('aarch64', 'arm64'):
        raise ControlError('仅供 ARM64 香橙派安装/运行')
    if controller is not None:
        verify_binary(controller)
    import yaml

    # Navigation callers pass a bundle-owned candidate config.  The project
    # root is retained for SDK discovery only and must not silently override
    # the installed navigation policy.
    selected = (
        Path(config_path) if config_path is not None else root / 'config/attitude_altitude.yaml'
    )
    with open(selected, encoding='utf8') as f:
        config = standby_config(yaml.safe_load(f))
    if not agent.is_file() or not os.access(agent, os.X_OK):
        raise ControlError('缺少离线 Agent: ' + str(agent))
    if shutil.which('fuser') is None:
        raise ControlError('缺少 fuser；未安装服务')
    import px4_msgs.msg as msgs
    from rclpy.type_support import check_for_type_support

    for name in (
        'BatteryStatus',
        'VehicleAttitude',
        'VehicleLocalPosition',
        'VehicleStatus',
        'VehicleLandDetected',
        'FailsafeFlags',
    ):
        check_for_type_support(getattr(msgs, name))
    return config


# 整个后台生命周期的资源所有者：同一时刻一把入口锁、一个原生控制器、一个传输恢复线程。
def supervise(
    root,
    agent_binary,
    folder,
    stopped,
    *,
    controller=None,
    config_factory=None,
    status_metadata=None,
    row_formatter=None,
    config_path=None,
):
    import yaml

    private_dir(folder)
    cache = Path.home() / '.cache/fc01_operator'
    private_dir(cache)
    log = rotating_logger(folder / 'standby.log', max_bytes=1024 * 1024)
    writer = StatusWriter(folder, log)
    last_message = None
    last_state, last_state_at, last_state_pid = {}, -math.inf, None
    diagnostics = {}
    metadata = dict(status_metadata or {})
    metadata['supervisor_pid'] = os.getpid()
    try:
        metadata['boot_id'] = Path('/proc/sys/kernel/random/boot_id').read_text().strip()
    except OSError:
        metadata['boot_id'] = ''

    # 没有新状态时可保留历史内容供诊断，但强制 feedback_current=false，且保留原 PID 与真实增长的年龄。
    def publish(row):
        nonlocal last_message, last_state, last_state_at, last_state_pid
        row = {**diagnostics, **row}
        now = time.monotonic()
        if row.get('state'):
            last_state = row['state']
            age = row.get('state_age_seconds')
            last_state_at = (
                now - age if type(age) in (int, float) and math.isfinite(age) else -math.inf
            )
            last_state_pid = row.get('state_controller_pid', row.get('controller_pid'))
        else:
            row.update(
                state=last_state,
                feedback_current=False,
                state_controller_pid=last_state_pid,
                state_age_seconds=(
                    max(0, now - last_state_at) if last_state_at != -math.inf else None
                ),
            )
        row = row_formatter(row) if row_formatter else row
        row = {**metadata, **row}
        writer.submit(row)
        key = row['status'] + ': ' + row['reason']
        if key != last_message:
            writer.message(key)
            last_message = key

    try:
        while not stopped.is_set():
            node = agent = lock = pump = channel = ipc = native_log = transport = None
            state, state_at, existing = {}, -math.inf, None
            failure = None
            try:
                # 与交互入口共用启动锁；锁被占用时等待现有会话，不能启动第二个控制器。
                lockfd = os.open(
                    cache / 'launcher.lock', os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW, 0o600
                )
                lock = os.fdopen(lockfd, 'w')
                try:
                    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
                except BlockingIOError as exc:
                    raise ControlError('旧入口正在运行；保留它，等待其退出') from exc
                if not wait_for_previous_controller(
                    agent_binary, cache, stopped, publish, writer.message
                ):
                    break
                preflight_options = {'controller': controller} if controller is not None else {}
                if config_path is not None:
                    preflight_options['config_path'] = config_path
                binary, existing = preflight(root, agent_binary, '/dev/ttyS2', **preflight_options)
                config = (
                    config_factory(root, agent_binary)
                    if config_factory
                    else validate(root, agent_binary, controller=binary)
                )
                with open(folder / 'runtime.yaml', 'w', encoding='utf8') as f:
                    yaml.safe_dump(config, f, allow_unicode=True, sort_keys=False)
                ipc = make_socket_path(cache / 'ipc')
                if existing is None:
                    # An Agent retained after shutdown has no pipe dependency on us.
                    agent = subprocess.Popen(
                        [
                            str(agent_binary),
                            'serial',
                            '-D',
                            '/dev/ttyS2',
                            '-b',
                            '115200',
                            '-v',
                            '0',
                        ],
                        stdin=subprocess.DEVNULL,
                        stdout=subprocess.DEVNULL,
                        stderr=subprocess.DEVNULL,
                        start_new_session=True,
                    )
                # Agent 的恢复周期与原生监控分开，传输中断不触发原生进程替换或重新执行任务。
                transport = AgentTransport(agent_binary, agent, existing, cache, writer.message)
                transport.start()
                # Open storage before the controller starts; the pump owns
                # this attempt's handlers after it starts reading stdout.
                # 先准备存储再启动原生；每次 LogPump 独占此次处理器，旧日志线程不能关闭下一次启动的新处理器。
                native_log = rotating_logger(folder / 'controller.log')
                node = subprocess.Popen(
                    [
                        str(binary),
                        '--ros-args',
                        '--disable-external-lib-logs',
                        '--params-file',
                        str(folder / 'runtime.yaml'),
                        '-p',
                        'socket_path:=' + str(ipc),
                    ],
                    stdin=subprocess.DEVNULL,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    start_new_session=True,
                )
                # Keep the last native outcome through failed reconnect checks;
                # a successfully started replacement begins a new IPC lifetime.
                diagnostics = {}
                # Each pump solely owns its handlers. An old pump delayed by
                # storage cannot close a replacement controller's logger.
                pump = LogPump(node.stdout, native_log)
                publish(
                    dict(
                        status='WAITING',
                        reason='控制器已启动，等待飞控反馈',
                        controller_pid=node.pid,
                        **transport.snapshot(),
                    )
                )
                channel = connect_controller(ipc, node)

                def observe(row):
                    row.update(transport.snapshot())
                    publish(row)

                # Agent loss is a transport failure. The same native process
                # handles feedback loss and fresh recovery evidence itself.
                state, state_at, failure = monitor(
                    channel, node, stopped, observe, diagnostics=diagnostics
                )
                if state:
                    last_state, last_state_at, last_state_pid = state, state_at, node.pid
                channel = None
            except Exception as exc:
                failure = str(exc)[:2000]
            finally:
                # 清理也要先请求并等待原生优雅退出；等待期间传输线程仍在，允许原生获得新的恢复证据。
                if node is not None and node.poll() is None:
                    NodeStop(node).request()
                    publish(
                        dict(
                            status='STOPPING',
                            reason='等待原生控制器优雅退出',
                            controller_pid=node.pid,
                        )
                    )
                    while node.poll() is None:
                        time.sleep(0.1)
                if transport is not None:
                    agent = transport.close()
                if channel is not None:
                    channel.sock.close()
                if node is not None:
                    diagnostics.update(controller_result(node))
                if pump is not None:
                    pump.join(timeout=3)
                elif native_log is not None:
                    for handler in native_log.handlers:
                        handler.close()
                if ipc is not None:
                    try:
                        ipc.unlink(missing_ok=True)
                        ipc.parent.rmdir()
                    except OSError as exc:
                        writer.message('保留本次 IPC 目录：%s', exc)
                # 仅自建 Agent 可进入停止判断；仍缺新鲜未解锁接地证据时保留，禁止用强杀完成退出。
                if agent is not None:
                    if finish_agent(
                        agent,
                        True,
                        node is not None and node.poll() == 0,
                        state,
                        state_at,
                        time.monotonic(),
                    ):
                        try:
                            agent.wait(timeout=5)
                        except subprocess.TimeoutExpired:
                            writer.message('Agent 尚未退出，未强制终止 PID=%s', agent.pid)
                    else:
                        writer.message(
                            '保留自建 Agent PID=%s；缺少新鲜未解锁接地退出证据', agent.pid
                        )
                if lock is not None:
                    lock.close()
            # 一次原生生命周期真正结束后才进入 30 秒重试；重试不恢复上次起飞任务或飞行授权。
            if not stopped.is_set():
                remaining = RETRY_SECONDS
                while remaining > 0 and not stopped.is_set():
                    reason = failure or '等待重新连接'
                    exit_reason = diagnostics.get('controller_exit_reason')
                    if exit_reason and exit_reason not in reason:
                        reason += '；' + exit_reason
                    publish(dict(status='WAITING', reason=reason, retry_in_seconds=remaining))
                    step = min(5, remaining)
                    if stopped.wait(step):
                        break
                    remaining -= step
        publish(dict(status='STOPPED', reason='后台待命已停止'))
    finally:
        writer.close()


# 旧版后台入口默认使用关闭动作门配置；信号只置停止事件，由监督循环按原生退出流程收尾。
def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--validate', action='store_true')
    parser.add_argument(
        '--root',
        type=Path,
        default=Path(os.environ.get('FC01_PROJECT_ROOT', str(Path.home() / PROJECT))),
    )
    parser.add_argument(
        '--agent', type=Path, default=Path.home() / 'fc01_agent_build_offline/MicroXRCEAgent'
    )
    args = parser.parse_args(argv)
    os.umask(0o077)
    root, agent = args.root.expanduser().resolve(), args.agent.expanduser().resolve()
    if args.validate:
        validate(
            root,
            agent,
            controller=root / 'install_r632/jikl_fc01_system/lib/jikl_fc01_system/fc01_system_node',
        )
        print('ROS / px4_msgs / ARM64 控制器 / 关闭动作门配置检查通过。')
        return 0
    stopped = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, lambda *_: stopped.set())
    supervise(root, agent, Path.home() / 'FC01_autostart', stopped)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
