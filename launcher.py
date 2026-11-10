#!/usr/bin/env python3
"""Offline Orange Pi entry for the installed R6.3.2 native controller."""
from __future__ import annotations

import argparse
from dataclasses import dataclass
from datetime import datetime, timezone
import errno
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import queue
import select
import shlex
import signal
import socket
import stat
import subprocess
import sys
import tempfile
import threading
import time
import uuid
import zipfile

from fc01_control import ControlError, DEFAULT_TAKEOFF_HEIGHT_M, Mission, readiness_issues, runtime_profile

PROJECT = 'FC01_ARM64_PORT_20261020/FC01_R632_P0_ATTITUDE_AUDIT_CANDIDATE'
BINARY_MARKERS = (b'R632_ACK176_STATE_CANDIDATE_V1', b'R632_SCHEDULED_TAKEOFF_CEILING_V1')
OPS = {'INTERACTIVE': 0, 'PING': 0, 'STATUS': 0, 'AUTHORITY_RC': 0,
       'ATTITUDE_READY': 0, 'TAKEOFF': 1, 'LAND': 0}
MAX_MESSAGE = 131072


class Display:
    """A vanished or flow-controlled terminal must never own the flight lease."""
    def __init__(self, stream):
        self.stream = stream
        self.messages = queue.Queue(maxsize=128)
        self.closed = False
        self.failed = False
        self.thread = threading.Thread(target=self._write, daemon=True)
        self.thread.start()

    def say(self, message):
        if self.closed or self.failed:
            return
        try:
            self.messages.put_nowait(str(message)[:4096]+'\n')
        except queue.Full:
            pass  # Bounded display backlog; evidence remains in the session log.

    def _write(self):
        while not self.closed or not self.messages.empty():
            try:
                message = self.messages.get(timeout=0.05)
            except queue.Empty:
                continue
            if not self.failed:
                try:
                    self.stream.write(message)
                    self.stream.flush()
                except (OSError, ValueError):
                    self.failed = True

    def close(self):
        self.closed = True
        # A blocked terminal writer is a daemon, not a reason to abandon recovery.
        self.thread.join(timeout=0.1)


def encode_command(seq, op, args):
    if type(seq) is not int or not 0 < seq < 2**64:
        raise ControlError('无效控制台序号')
    if op not in OPS or len(args) != OPS[op]:
        raise ControlError('入口不支持此命令/参数: '+str(op))
    if any(type(x) not in (float, int) or not math.isfinite(x) for x in args):
        raise ControlError('命令参数必须为有限数字')
    return ('%d %s%s\n' % (seq, op, ''.join(' %.9g' % x for x in args))).encode('ascii')


class ChannelError(ControlError):
    """A terminal IPC condition, independent of the controller's exit status."""
    def __init__(self, kind, message, error_number=None):
        super().__init__(kind+': '+message)
        self.kind = kind
        self.errno = error_number

    def as_dict(self):
        result = dict(kind=self.kind, message=str(self))
        if self.errno is not None:
            result['errno'] = self.errno
        return result


class Channel:
    def __init__(self, sock):
        self.sock = sock
        self.sock.setblocking(False)
        self.seq = 0
        self.buf = b''
        self.eof = False
        self.read_error = None

    def send(self, op, args):
        wire = encode_command(self.seq+1, op, args)
        # Consume a sequence before writing: a partial write is never replayed.
        self.seq += 1
        view = memoryview(wire)
        deadline = time.monotonic()+0.3
        while view:
            left = deadline-time.monotonic()
            if left <= 0 or not select.select([], [self.sock], [], max(0, left))[1]:
                raise ControlError('控制台写入超时；禁止重放命令')
            try:
                count = self.sock.send(view)
            except BlockingIOError:
                continue
            if not count:
                raise ControlError('控制台写入断开')
            view = view[count:]
        return self.seq

    def read(self):
        if self.read_error is not None:
            raise self.read_error
        objects = []
        def terminal(error):
            self.read_error = error
            # Deliver every valid preceding record before exposing the failure.
            if objects:
                return objects
            raise error
        while True:
            try:
                chunk = self.sock.recv(65536)
            except BlockingIOError:
                break
            except OSError as exc:
                return terminal(ChannelError('socket_error', str(exc), exc.errno))
            if not chunk:
                self.eof = True
                if self.buf:
                    return terminal(ChannelError('truncated_json', '控制台关闭时存在未完成的 JSON 消息'))
                return terminal(ChannelError('clean_eof', '控制台连接已关闭；没有残缺消息'))
            self.buf += chunk
            while b'\n' in self.buf:
                line, self.buf = self.buf.split(b'\n', 1)
                if len(line) > MAX_MESSAGE:
                    return terminal(ChannelError('invalid_message', '控制器消息超长'))
                try:
                    obj = json.loads(line, parse_constant=lambda _: (_ for _ in ()).throw(ValueError()))
                except (ValueError, UnicodeDecodeError):
                    return terminal(ChannelError('invalid_message', '控制器消息格式错误'))
                if not isinstance(obj, dict):
                    return terminal(ChannelError('invalid_message', '控制器消息必须为 JSON 对象'))
                objects.append(obj)
            if len(self.buf) > MAX_MESSAGE:
                return terminal(ChannelError('invalid_message', '控制器消息超长'))
        return objects


@dataclass(frozen=True)
class Process:
    pid: int
    uid: int
    argv: tuple[str, ...]


def process_list():
    result = []
    for entry in Path('/proc').iterdir():
        if not entry.name.isdecimal():
            continue
        try:
            argv = tuple(x.decode(errors='replace') for x in (entry/'cmdline').read_bytes().split(b'\0') if x)
            if argv:
                result.append(Process(int(entry.name), entry.stat().st_uid, argv))
        except (FileNotFoundError, ProcessLookupError):
            continue
        except PermissionError as exc:
            raise ControlError('无法检查进程所有权；未启动第二个控制器') from exc
    return result


def option(argv, *names):
    values = []
    for i, arg in enumerate(argv):
        if arg in names and i+1 < len(argv):
            values.append(argv[i+1])
    return values[0] if len(values) == 1 else None


def choose_agent(processes, owners, device, uid):
    candidates = [p for p in processes if Path(p.argv[0]).name == 'MicroXRCEAgent'
                  and 'serial' in p.argv and option(p.argv, '-D', '--dev') == device]
    if len(candidates) > 1:
        raise ControlError('发现多个此 UART 的 Agent，保持原进程，拒绝启动')
    if candidates:
        p = candidates[0]
        if p.uid != uid or option(p.argv, '-b', '--baudrate') != '115200':
            raise ControlError('现有 Agent 用户/波特率不匹配，需要同用户 115200')
        if owners - {p.pid}:
            raise ControlError('UART 还被其他进程占用: '+str(sorted(owners-{p.pid})))
        return p
    if owners:
        raise ControlError('UART 被未知程序占用，未停止它: '+str(sorted(owners)))
    return None


def uart_owners(device):
    try:
        p = subprocess.run(['fuser', device], capture_output=True, text=True, timeout=4)
    except (FileNotFoundError, subprocess.TimeoutExpired) as exc:
        raise ControlError('需要已安装的 fuser 来确认串口所有权') from exc
    if p.returncode not in (0, 1) or (p.returncode == 1 and p.stderr.strip()):
        raise ControlError('无法确认串口所有权: '+p.stderr.strip()[:200])
    if any(not x.isdecimal() for x in p.stdout.split()):
        raise ControlError('fuser 返回内容无法辨认')
    return {int(x) for x in p.stdout.split()}


def verify_binary(path):
    if not path.is_file() or not os.access(path, os.X_OK):
        raise ControlError('缺少可执行的已安装控制器: '+str(path))
    data = path.read_bytes()
    if len(data) < 64 or data[:6] != b'\x7fELF\x02\x01' or int.from_bytes(data[18:20], 'little') != 183:
        raise ControlError('控制器不是 Linux ARM64 ELF')
    if any(marker not in data for marker in BINARY_MARKERS):
        raise ControlError('控制器缺少当前 ACK176/起飞推力上限标识；已保留现有安装')


def private_dir(path):
    if path.is_symlink():
        raise ControlError('私有运行目录不能是符号链接: '+str(path))
    path.mkdir(mode=0o700, parents=True, exist_ok=True)
    st = path.stat()
    if st.st_uid != os.getuid() or st.st_mode & 0o077:
        raise ControlError('运行目录需要当前用户所有且权限 0700: '+str(path))


def make_socket_path(root):
    private_dir(root)
    folder = Path(tempfile.mkdtemp(prefix='s_', dir=root))
    path = folder/'control.sock'
    if len(os.fsencode(path)) > 103:
        raise ControlError('Unix socket 路径过长: '+str(path))
    return path


class Evidence:
    NAMES = ('metadata.json', 'runtime.yaml', 'controller.log', 'agent.log',
             'console.jsonl', 'telemetry.csv', 'summary.json')

    def __init__(self, folder, metadata):
        self.folder = folder
        folder.mkdir(mode=0o700, parents=True, exist_ok=False)
        self.first_failure = None
        self.last_state = {}
        self.last_state_at = -math.inf
        self.truncated = []
        self.write_json('metadata.json', metadata)
        self.events = open(folder/'console.jsonl', 'x', buffering=1, encoding='utf8')

    def write_json(self, name, obj):
        with open(self.folder/name, 'w', encoding='utf8') as f:
            json.dump(obj, f, ensure_ascii=False, indent=2, allow_nan=False)
            f.write('\n')

    def failure(self, text):
        if self.first_failure is None:
            self.first_failure = {'kind': 'launcher', 'message': str(text)}

    def record(self, obj, now):
        try:
            self.events.write(json.dumps({'host_monotonic': now, 'data': obj}, ensure_ascii=False, allow_nan=False)+'\n')
        except OSError as exc:
            self.failure('日志写入失败: '+str(exc))
        if obj.get('kind') in ('reply', 'state') and 'phase' in obj:
            self.last_state, self.last_state_at = obj, now
        if self.first_failure is None and (obj.get('accepted') is False or obj.get('phase') == 'SAFETY'):
            self.first_failure = obj

    def command(self, seq, op, args, now):
        self.record({'kind': 'command', 'seq': seq, 'op': op, 'args': args}, now)

    def checkpoint(self, outcome):
        # Only a small current-state JSON while the node is alive. Archive
        # scanning/compression is deliberately deferred until native exit.
        try:
            self.write_json('summary.json', {'outcome': outcome, 'first_failure': self.first_failure,
                            'last_state': self.last_state, 'truncated_files': self.truncated,
                            'hardware_flight_verified_by_this_bundle': False})
        except OSError as exc:
            self.failure('状态摘要写入失败: '+str(exc))

    def report(self, outcome):
        self.events.flush()
        self.write_json('summary.json', {'outcome': outcome, 'first_failure': self.first_failure,
                        'last_state': self.last_state, 'truncated_files': self.truncated,
                        'hardware_flight_verified_by_this_bundle': False})
        path = self.folder/'report.zip'
        self.truncated.clear()
        limit = 8*1024*1024
        with zipfile.ZipFile(path, 'w', zipfile.ZIP_DEFLATED) as z:
            for name in self.NAMES:
                p = self.folder/name
                if not p.is_file() or p.is_symlink():
                    continue
                with open(p, 'rb') as f:
                    size = os.fstat(f.fileno()).st_size
                    if size > limit:
                        self.truncated.append(name)
                        data = f.read(limit//4)
                        f.seek(-limit*3//4, os.SEEK_END)
                        data += b'\n[REPORT COPY TRUNCATED; ORIGINAL RETAINED]\n'+f.read(limit*3//4)
                    else:
                        data = f.read(limit)
                if name != 'summary.json':
                    z.writestr(name, data)
            self.write_json('summary.json', {'outcome': outcome, 'first_failure': self.first_failure,
                            'last_state': self.last_state, 'truncated_files': self.truncated,
                            'hardware_flight_verified_by_this_bundle': False})
            z.write(self.folder/'summary.json', 'summary.json')
        return path


class NodeStop:
    def __init__(self, node):
        self.node, self.requested = node, False

    def request(self):
        if not self.requested and self.node.poll() is None:
            self.requested = True
            try:
                self.node.send_signal(signal.SIGTERM)
            except ProcessLookupError:
                pass


def finish_agent(agent, owned, node_exited, state, state_at, now):
    safe = (node_exited and 0 <= now-state_at <= 1 and
            state.get('phase') in ('IDLE', 'SAFE_DISARMED') and
            state.get('armed') is False and state.get('landed') is True and
            state.get('status_fresh') is True and state.get('land_fresh') is True)
    if owned and agent is not None and safe:
        if agent.poll() is None:
            agent.terminate()
        return True
    return False


def state_line(state):
    h = state.get('relative_height_m')
    height = '未知' if type(h) not in (float, int) else '%.2f m' % h
    return ('[%s] armed=%s landed=%s 相对高度=%s 原因=%s' %
            (state.get('phase'), state.get('armed'), state.get('landed'), height, state.get('reason', '')))


def connect_controller(path, node, timeout=12):
    end = time.monotonic()+timeout
    while time.monotonic() < end:
        if node.poll() is not None:
            raise ControlError('控制器在控制台就绪前退出，查看自动报告 controller.log')
        s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        try:
            s.connect(str(path))
            return Channel(s)
        except OSError as exc:
            s.close()
            if exc.errno not in (errno.ENOENT, errno.ECONNREFUSED):
                raise
        time.sleep(0.1)
    raise ControlError('等待控制器 socket 超时，未删除任何旧 socket')


def run_console(channel, node, evidence, params, flight, seconds, stop_signal):
    display = Display(sys.stdout)
    say = display.say
    mission = Mission(params['aa_max_height'])
    stop = NodeStop(node)
    closing = False
    started = time.monotonic()
    last_ping = -math.inf
    last_print = -math.inf
    printed_phase = None
    old_error = None
    outcome = 'check_blocked'
    send_failure = None
    healthy_seen = False

    def send(op, values):
        seq = channel.send(op, values)
        evidence.command(seq, op, values, time.monotonic())
        return seq

    def close_session(reason):
        nonlocal closing, outcome
        if closing:
            return
        closing = True
        mission.cancel()
        outcome = reason
        # Native LAND owns release/descent. Never synthesize a disarm here.
        if flight and channel is not None:
            try:
                send('LAND', [])
            except (OSError, ControlError) as exc:
                evidence.failure(exc)
        stop.request()
        evidence.checkpoint(outcome+'; waiting_native_shutdown')
        say('正在等待原生控制器完成退出/恢复；Agent 和心跳保持运行。')

    try:
        send('INTERACTIVE', [])
        send('STATUS', [])
        say(f'命令：takeoff [高度m，默认{DEFAULT_TAKEOFF_HEIGHT_M}] | land | status | quit' if flight else '只检查：动作门关闭，正在收集实时状态……')
        while node.poll() is None:
            now = time.monotonic()
            if stop_signal[0]:
                close_session('operator_signal')
            if not flight and now-started >= seconds:
                fresh = 0 <= now-evidence.last_state_at <= 1
                ok = fresh and not readiness_issues(evidence.last_state)
                close_session('check_pass' if ok and healthy_seen else 'check_blocked')
            if channel is not None and now-last_ping >= 0.2:
                try:
                    send('PING', [])
                    last_ping = now
                except (ControlError, OSError) as exc:
                    send_failure = str(exc)
            readers = ([channel.sock] if channel is not None else [])
            if flight and not closing:
                readers.append(sys.stdin)
            ready = select.select(readers, [], [], 0.05)[0] if readers else []
            if not readers:
                time.sleep(0.05)
            if channel is not None and channel.sock in ready:
                try:
                    objects = channel.read()
                    for obj in objects:
                        evidence.record(obj, now)
                        if obj.get('kind') in ('reply', 'state') and 'phase' in obj:
                            mission.observe(obj, now)
                            healthy_seen |= not readiness_issues(obj)
                            phase = obj.get('phase')
                            if phase != printed_phase or now-last_print >= (3 if closing else 5):
                                say(state_line(obj))
                                if not closing and not mission.submitted:
                                    issues = readiness_issues(obj)
                                    if issues:
                                        say('等待条件：'+', '.join(issues))
                                printed_phase, last_print = phase, now
                        if obj.get('kind') == 'reply':
                            pending = mission.pending
                            mission.reply(obj, now)
                            if (pending and obj.get('seq') == pending[0]) or obj.get('accepted') is False:
                                say(('[接受] ' if obj.get('accepted') is True else '[拒绝] ')+str(obj.get('message')))
                    if channel.read_error is not None:
                        raise channel.read_error
                except (ControlError, OSError) as exc:
                    send_failure = str(exc)
            if send_failure is not None:
                evidence.failure(send_failure)
                mission.fail(send_failure)
                say(send_failure+'；转入原生退出/恢复流程。')
                if channel is not None:
                    channel.sock.close()
                channel = None
                send_failure = None
                close_session('console_lost')
            if flight and not closing and sys.stdin in ready:
                text = sys.stdin.readline()
                if not text:
                    close_session('operator_eof')
                else:
                    try:
                        parts = shlex.split(text)
                        if not parts:
                            continue
                        if parts[0] == 'takeoff' and len(parts) in (1, 2):
                            mission.request_takeoff(float(parts[1]) if len(parts) == 2 else DEFAULT_TAKEOFF_HEIGHT_M, now)
                            say('准备起飞 %.2f m；核对 RC 和实时反馈后由原生控制器执行一次 TAKEOFF。' % mission.target)
                        elif parts == ['land']:
                            mission.cancel()
                            send('LAND', [])
                            say('已请求原生降落/地面释放。')
                        elif parts == ['status']:
                            send('STATUS', [])
                            say(state_line(evidence.last_state))
                            say('等待条件：'+(', '.join(readiness_issues(evidence.last_state)) or '地面反馈已齐备'))
                        elif parts == ['quit']:
                            close_session('operator_quit')
                        else:
                            say(f'命令：takeoff [{DEFAULT_TAKEOFF_HEIGHT_M}] | land | status | quit')
                    except (ControlError, ValueError) as exc:
                        evidence.failure(exc)
                        say('未执行：'+str(exc))
            if flight and not closing and channel is not None:
                action = mission.poll(now)
                if action:
                    op, values = action
                    seq = send(op, values)
                    if mission.stage == 'ISSUED_'+op:
                        mission.sent(seq, op)
                if mission.error and mission.error != old_error:
                    old_error = mission.error
                    evidence.failure(old_error)
                    say('起飞流程停止：'+old_error)
                    evidence.checkpoint('takeoff_refused')
            if closing and now-last_print >= 10:
                say('仍在等待原生恢复完成；如飞机仍在空中，使用已验证的 RC 接管。未强制结束控制器。')
                last_print = now
                evidence.checkpoint(outcome+'; waiting_native_shutdown')
        # The process may exit immediately after writing its final ground state.
        # Drain already queued bytes before deciding whether an owned Agent can stop.
        if channel is not None:
            try:
                for obj in channel.read():
                    evidence.record(obj, time.monotonic())
            except (ControlError, OSError):
                pass
        if not closing:
            outcome = 'native_exited_unexpectedly'
            evidence.failure('控制器意外退出，原生退出码='+str(node.returncode))
        return outcome
    except (Exception, KeyboardInterrupt) as exc:
        evidence.failure(exc)
        close_session('launcher_error')
        # Do not abandon a live controller in its recovery loop after an UI error.
        while node.poll() is None:
            if channel is not None:
                try:
                    send('PING', [])
                    if select.select([channel.sock], [], [], 0.2)[0]:
                        for obj in channel.read():
                            evidence.record(obj, time.monotonic())
                        if channel.read_error is not None:
                            raise channel.read_error
                except (OSError, ControlError):
                    channel.sock.close()
                    channel = None
            else:
                time.sleep(0.2)
        return 'launcher_error'
    finally:
        if channel is not None:
            channel.sock.close()
        display.close()


def preflight(root, agent_binary, device, *, controller=None, config_path=None):
    if platform.machine().lower() not in ('aarch64', 'arm64'):
        raise ControlError('本入口只在实际 ARM64 香橙派上运行；本机架构='+platform.machine())
    if os.environ.get('ROS_DOMAIN_ID', '0') != '0' or os.environ.get('ROS_LOCALHOST_ONLY', '0') != '0':
        raise ControlError('当前固定使用 ROS_DOMAIN_ID=0、ROS_LOCALHOST_ONLY=0')
    node = controller if controller is not None else root/'install_r632/jikl_fc01_system/lib/jikl_fc01_system/fc01_system_node'
    verify_binary(node)
    # The navigation bundle may be installed beside an older SDK checkout.
    # Callers can provide the bundle-owned candidate config; the project root
    # remains an SDK/runtime dependency and is never used as a source of
    # navigation policy in that path.
    selected_config = Path(config_path) if config_path is not None else root/'config/attitude_altitude.yaml'
    if not selected_config.is_file():
        raise ControlError('缺少原配置 attitude_altitude.yaml: '+str(selected_config))
    if not Path(device).exists() or not stat.S_ISCHR(Path(device).stat().st_mode) or not os.access(device, os.R_OK | os.W_OK):
        raise ControlError('串口不存在/不可读写: '+device)
    processes = process_list()
    others = [p.pid for p in processes if Path(p.argv[0]).name == 'fc01_system_node']
    if others:
        raise ControlError('已有控制器运行，保持它，不启动第二个: '+str(others))
    existing = choose_agent(processes, uart_owners(device), device, os.getuid())
    if existing is None and (not agent_binary.is_file() or not os.access(agent_binary, os.X_OK)):
        raise ControlError('没有可复用的 Agent，且未找到离线 Agent: '+str(agent_binary))
    try:
        import px4_msgs.msg as msgs
        from rclpy.type_support import check_for_type_support
        for name in ('BatteryStatus', 'VehicleAttitude', 'VehicleLocalPosition', 'VehicleStatus', 'VehicleLandDetected', 'FailsafeFlags'):
            check_for_type_support(getattr(msgs, name))
    except (ImportError, AttributeError, RuntimeError) as exc:
        raise ControlError('px4_msgs 类型支持未加载；请通过 start.sh 自动加载 install_r632 环境') from exc
    return node, existing


def main(argv=None):
    parser = argparse.ArgumentParser(description='FC01 香橙派单入口：检查 / 交互起飞、定高、降落')
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--flight', action='store_true', help='明确选择现有候选参数和 RC 接管路径；仍需交互 takeoff')
    mode.add_argument('--check', action='store_true', help='默认：动作门关闭，只检查实时反馈')
    parser.add_argument('--root', type=Path, default=Path(os.environ.get('FC01_PROJECT_ROOT', str(Path.home()/PROJECT))))
    parser.add_argument('--agent', type=Path, default=Path.home()/'fc01_agent_build_offline/MicroXRCEAgent')
    parser.add_argument('--check-seconds', type=float, default=15)
    args = parser.parse_args(argv)
    if not math.isfinite(args.check_seconds) or not 5 <= args.check_seconds <= 60:
        parser.error('--check-seconds 必须在 [5,60] 秒内')
    os.umask(0o077)
    display = Display(sys.stdout)
    say = display.say
    run_id = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')+'_'+uuid.uuid4().hex[:6]
    folder = Path.home()/'FC01_control_runs'/run_id
    evidence = Evidence(folder, {'mode': 'flight' if args.flight else 'check', 'domain': '0',
                                'project': str(args.root), 'device': '/dev/ttyS2', 'baud': 115200,
                                'entry_version': '20261027.2'})
    node = agent = channel = None
    owned_agent = False
    lock = None
    files = []
    stopped = [False]
    outcome = 'preflight_refused'
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, lambda *_: stopped.__setitem__(0, True))
    try:
        if args.flight and not sys.stdin.isatty():
            raise ControlError('--flight 需要现场交互终端；禁止管道自动起飞')
        root = args.root.expanduser().resolve()
        cache = Path.home()/'.cache/fc01_operator'
        private_dir(cache)
        lock_path = cache/'launcher.lock'
        lockfd = os.open(lock_path, os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW, 0o600)
        lock = os.fdopen(lockfd, 'w')
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise ControlError('另一个本入口仍在运行；保持现有会话') from exc
        binary, existing = preflight(root, args.agent, '/dev/ttyS2')
        import yaml
        with open(root/'config/attitude_altitude.yaml', encoding='utf8') as f:
            config = runtime_profile(yaml.safe_load(f), args.flight)
        params = config['fc01_system']['ros__parameters']
        config_path = folder/'runtime.yaml'
        # Keep the standalone legacy console compatible with native launch
        # provenance checks. Navigation service mode selection is separate;
        # this path remains an explicitly interactive --flight session.
        params.update(fc01_runtime_mode='flight-test' if args.flight else 'diagnostic',
                      fc01_entry_version='launcher-legacy', fc01_config_path=str(config_path.resolve()),
                      fc01_config_source_path=str((root/'config/attitude_altitude.yaml').resolve()))
        with open(config_path, 'x', encoding='utf8') as f:
            yaml.safe_dump(config, f, allow_unicode=True, sort_keys=False)
        say('会话配置：20 Hz；UART 115200；定高上限 %.1f m；最长 %.1f s；倾角上限 %.1f°。' %
            (params['aa_max_height'], params['aa_max_flight_seconds'], params['aa_max_tilt_deg']))
        say('候选参数原样保留：hover=%.3f，起飞余量=%.3f；本包没有完成实机标定。' %
            (params['aa_hover_thrust'], params['aa_takeoff_reserve_thrust']))
        if args.flight:
            say('--flight 已开启本次会话动作门并选择 RC 接管路径；输入 takeoff 才请求解锁起飞。')
        ipc = make_socket_path(cache/'ipc')
        if existing is None:
            log = open(folder/'agent.log', 'xb', buffering=0)
            files.append(log)
            agent = subprocess.Popen([str(args.agent), 'serial', '-D', '/dev/ttyS2', '-b', '115200', '-v', '4'],
                                     stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            owned_agent = True
            say('启动离线 Agent，PID='+str(agent.pid))
        else:
            say('复用现有 115200 Agent，PID='+str(existing.pid)+'；退出时保持它运行。')
        evidence.write_json('metadata.json', {'mode': 'flight' if args.flight else 'check', 'domain': '0',
                'project': str(root), 'device': '/dev/ttyS2', 'baud': 115200, 'socket': str(ipc),
                'agent_owned': owned_agent, 'agent_pid': agent.pid if owned_agent else existing.pid,
                'binary_sha256': hashlib.sha256(binary.read_bytes()).hexdigest(), 'entry_version': '20261027.2'})
        log = open(folder/'controller.log', 'xb', buffering=0)
        files.append(log)
        node = subprocess.Popen([str(binary), '--ros-args', '--params-file', str(config_path),
                                '-p', 'socket_path:='+str(ipc), '-p', 'csv_path:='+str(folder/'telemetry.csv')],
                                stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        channel = connect_controller(ipc, node)
        outcome = run_console(channel, node, evidence, params, args.flight, args.check_seconds, stopped)
        channel = None
    except (Exception, KeyboardInterrupt) as exc:
        evidence.failure(exc)
        say('未继续执行：'+str(exc))
        if node is not None and node.poll() is None:
            NodeStop(node).request()
            say('原生控制器仍在退出/恢复；等待它自行完成，保留 Agent。')
            evidence.checkpoint('launch_error; waiting_native_shutdown')
            while node.poll() is None:
                time.sleep(0.2)
        outcome = 'launch_error'
    finally:
        if channel is not None:
            channel.sock.close()
        clean_exit = (node is not None and node.poll() == 0 and
                      outcome in ('check_pass', 'check_blocked', 'operator_quit', 'operator_eof', 'operator_signal'))
        if owned_agent and not finish_agent(agent, True, clean_exit,
                                            evidence.last_state, evidence.last_state_at, time.monotonic()):
            say('未获得新鲜的未解锁接地证据，保持自建 Agent PID='+str(agent.pid)+' 运行。')
        for f in files:
            f.close()
        if lock is not None:
            lock.close()
        try:
            report = evidence.report(outcome)
            say('自动报告：'+str(report))
        except OSError as exc:
            say('报告写入失败：'+str(exc)+'；会话目录：'+str(folder))
            outcome = 'report_write_error'
        finally:
            evidence.events.close()
            display.close()
    return 0 if outcome in ('check_pass', 'operator_quit', 'operator_eof', 'operator_signal') else 1


if __name__ == '__main__':
    sys.exit(main())
