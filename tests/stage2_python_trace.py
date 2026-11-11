"""第二阶段动态对照工人：在独立进程导入指定源码树，所有外部动作均由fixture接管。"""

from __future__ import annotations

import argparse
import contextlib
from datetime import datetime, timezone
import hashlib
import importlib
import io
import json
import math
import os
from pathlib import Path
import shutil
import sys
import tempfile
import types
from unittest.mock import patch
import zipfile


class FrozenDateTime(datetime):
    """为文件名与采集元数据提供固定墙钟，不改变业务单调时钟的单位。"""

    @classmethod
    def now(cls, tz=None):
        value = cls(2026, 11, 11, 12, 0, 0)
        return value if tz is None else value.replace(tzinfo=timezone.utc).astimezone(tz)


def render_trace(root, folder):
    """逐字记录状态渲染，覆盖身份/新鲜度/故障恢复和未知字段；不读服务。"""
    import navigation as m

    info = dict(ActiveState='active', MainPID='41', boot_id='fixed-boot')
    base = dict(
        status='READY_WAIT_ARM',
        reason='base',
        supervisor_pid=41,
        boot_id='fixed-boot',
        updated_unix=1000.0,
        state_age_seconds=0.1,
        feedback_current=True,
        state=dict(
            phase='OFFBOARD_READY',
            nav_state=15,
            armed=False,
            relative_height_m=0.0,
            status_fresh=True,
            local_fresh=True,
            land_fresh=True,
            status_receive_age_s=0.0,
            local_receive_age_s=0.0,
        ),
    )
    base['state']['source_timing'] = {
        name: dict(
            source_reason='VALID',
            receipt_reason='VALID',
            timeout_s=timeout,
            envelope_age_s=0.02,
            sample_age_s=0.02,
            receipt_age_s=0.02,
            sample_progress_age_s=0.02,
        )
        for name, timeout in [
            ('status', 2.5),
            ('land', 2.5),
            ('local', 0.5),
            ('attitude', 0.5),
            ('flags', 2.5),
            ('battery', 1.0),
        ]
    }
    rows = []
    import copy

    for active in ['active', 'activating', 'inactive']:
        for age in [None, -1.0, 0.0, 0.5, 4.0, float('nan')]:
            for phase in ['OFFBOARD_READY', 'SAFETY', 'HOLDING', '']:
                row = copy.deepcopy(base)
                row['state_age_seconds'] = age
                row['state'].update(
                    phase=phase,
                    reason='OPERATOR_TAKEOVER_LATCHED',
                    rc_first_fault_reason='first',
                    rc_latest_fault_reason='latest',
                    rc_recovery_block_reason='CORE',
                    rc_reprepare_block_reason='RC_CORE_RESTART_CONDITIONS_NOT_MET',
                    rc_recovery_elapsed_s=1.0,
                    rc_recovery_required_s=2.0,
                    rc_recovery_ready=False,
                    rc_recovery_event_blocked=True,
                    rc_fault_recoverable=True,
                    rc_target_height=1.5,
                    takeoff_stage='HOLD',
                    liftoff_confirmed=False,
                    last_target_local_thrust_z=-0.5,
                )
                rows.append(m.render_status(row, {**info, 'ActiveState': active}, now=1000.2))
    for changed in [
        dict(supervisor_pid=99),
        dict(updated_unix=9990.0),
        dict(state=None),
        dict(boot_id='old'),
        dict(status='HEIGHT_HOLD'),
        dict(ipc_error={'kind': 'bad', 'message': 'tail'}),
    ]:
        rows.append(m.render_status({**base, **changed}, info, now=1000.2))
    assert any('[FAULT]' in row for row in rows)
    assert any('实际模式：' in row for row in rows)
    return rows


def report_trace(root, folder):
    """比较ZIP每个成员和采集索引，包含缺文件、裁尾、符号链接和只读命令失败。"""
    import navigation_report as m

    home = folder / 'home'
    entry = home / 'entry'
    runtime = home / 'running'
    evidence = home / 'FC01_navigation'
    for path in [entry, runtime / 'native/config', evidence]:
        path.mkdir(parents=True, exist_ok=True)
    row = dict(
        boot_id='fixed',
        supervisor_pid=41,
        controller_pid=42,
        runtime_mode='diagnostic',
        config_source_path=str(runtime / 'native/config/diagnostic.yaml'),
        state={'phase': 'SAFETY', 'reason': 'OPERATOR_TAKEOVER_LATCHED'},
    )
    files = {
        evidence / 'status.json': json.dumps(row).encode(),
        evidence / 'runtime.yaml': b'fixed: false\n',
        evidence / 'controller.log': b'a' * 1000 + b'last\n',
        entry / 'FC01_RELEASE.json': b'{"version":1}',
        runtime / 'HOST_VERIFICATION.json': b'{}',
        evidence / 'probe.json': b'{"ok":false}',
    }
    for path, data in files.items():
        path.write_bytes(data)
        os.utime(path, ns=(1_000_000_000, 1_000_000_000))
    (evidence / 'standby.log').symlink_to(evidence / 'controller.log')
    commands = []

    def runner(argv, **kwargs):
        commands.append([argv, kwargs])
        if 'journalctl' in argv[0]:
            raise OSError('fixture journal unavailable')
        return types.SimpleNamespace(stdout='active\n', stderr='fixture\n', returncode=3)

    output = io.StringIO()
    with (
        patch.object(m, 'datetime', FrozenDateTime),
        patch.object(m.time, 'monotonic', return_value=50.0),
        patch.object(m, 'MAX_LOG', 64),
        contextlib.redirect_stdout(output),
    ):
        target = m.collect_report(home, entry=entry, runner=runner)
    with zipfile.ZipFile(target) as archive:
        members = [
            [name, hashlib.sha256(archive.read(name)).hexdigest()] for name in archive.namelist()
        ]
    return dict(
        commands=commands,
        members=members,
        stdout=output.getvalue(),
        mode=target.stat().st_mode & 0o777,
    )


def service_trace(root, folder):
    """只调用被mock的入口动作，比较参数校验、配置工厂和监督调用顺序；不运行控制器。"""
    import navigation_service as m

    events = []
    home = folder / 'home'
    home.mkdir(mode=0o700)
    p = dict(
        aa_hover_thrust=0.50,
        aa_takeoff_reserve_thrust=0.04,
        rc_target_height=1.5,
        flight_actions_enabled=True,
        arm_enabled=False,
        takeoff_enabled=True,
        rc_navigation_enabled=True,
        visual_navigation_enabled=False,
    )

    def profile(*args, **kwargs):
        events.append(['profile', [str(a) for a in args], {k: str(v) for k, v in kwargs.items()}])
        return {'fc01_system': {'ros__parameters': dict(p)}}

    def supervise(*args, **kwargs):
        events.append(
            [
                'supervise',
                [str(a) if not hasattr(a, 'is_set') else 'stop-event' for a in args],
                {
                    k: str(v) if isinstance(v, Path) else v
                    for k, v in kwargs.items()
                    if k not in ('config_factory', 'row_formatter')
                },
            ]
        )
        kwargs['config_factory'](args[0], args[1])

    import yaml

    closed = {
        key: False
        for key in [
            'flight_enabled',
            'flight_actions_enabled',
            'arm_enabled',
            'takeoff_enabled',
            'rc_navigation_enabled',
            'bench_arm_only_enabled',
            'visual_navigation_enabled',
            'flight_parameters_confirmed',
            'rc_takeover_confirmed',
            'autonomous_failsafe_confirmed',
        ]
    }
    diagnostic = home / 'diagnostic.yaml'
    diagnostic.write_text(yaml.safe_dump({'fc01_system': {'ros__parameters': closed}}))
    invalid = home / 'invalid.yaml'
    invalid.write_text(
        yaml.safe_dump({'fc01_system': {'ros__parameters': {**closed, 'takeoff_enabled': True}}})
    )
    malformed = home / 'malformed.yaml'
    malformed.write_text('{}')
    checker = folder / 'entry/install/lib/jikl_fc01_system/fc01_config_check'
    checker.parent.mkdir(parents=True)
    checker.write_text('fixture checker')
    checker.chmod(0o700)

    def native_check(argv, **kwargs):
        events.append(['native-check', argv, kwargs])
        return types.SimpleNamespace(returncode=3)

    stdout = io.StringIO()
    stderr = io.StringIO()
    with (
        patch.object(m.subprocess, 'run', side_effect=native_check),
        patch.object(m.Path, 'home', return_value=home),
        patch.object(m, 'ENTRY', folder / 'entry'),
        patch.object(m, 'load_profile', side_effect=profile),
        patch.object(
            m,
            'check_controller',
            side_effect=lambda *a, **k: events.append(['controller-check']) or folder / 'binary',
        ),
        patch.object(m, 'controller_path', return_value=folder / 'binary'),
        patch.object(
            m, 'candidate_config_path', side_effect=lambda mode: folder / (mode + '.yaml')
        ),
        patch.object(m.standby, 'supervise', side_effect=supervise),
        patch.object(
            m.signal, 'signal', side_effect=lambda sig, handler: events.append(['signal', int(sig)])
        ),
        patch.dict(os.environ, {'FC01_PROJECT_ROOT': str(folder / 'sdk')}),
        contextlib.redirect_stdout(stdout),
        contextlib.redirect_stderr(stderr),
    ):
        results = []
        for argv in [
            ['validate-env'],
            ['prepare-config', '--file', str(home / 'runtime.yaml')],
            ['run', '--flight-test', '--confirm-flight-test'],
            ['run', '--flight-test'],
            ['check-config', '--file', str(diagnostic)],
            ['check-config', '--file', str(invalid)],
            ['check-config', '--file', str(malformed)],
            ['check-config', '--file', str(diagnostic), '--flight-test', '--confirm-flight-test'],
        ]:
            try:
                results.append(m.main(argv))
            except SystemExit as exc:
                results.append(['SystemExit', exc.code])
            except Exception as exc:
                results.append([type(exc).__name__, str(exc)])
    assert results[:5] == [0, 0, 0, ['SystemExit', 2], 0], results
    assert results[5][0] == results[6][0] == 'ControlError' and results[7] == 3, results
    return dict(
        events=events,
        results=results,
        stdout=stdout.getvalue(),
        stderr=stderr.getvalue(),
        config=(home / 'runtime.yaml').read_text(),
    )


def activation_trace(root, folder):
    """复用既有全mock事务fixture，比较启用/失败恢复每一步与最终证据；无systemd调用。"""
    sys.path.insert(1, str(root / 'tests'))
    import test_navigation_deployment_activation as f

    m = f.m
    results = []
    for scenario in [
        'normal',
        'install-failure',
        'probe-failure',
        'old-stuck',
        'verify-failure',
        'repeat',
    ]:
        case = folder / scenario
        case.mkdir()
        fixture = f.ActivationFixture(case, standby=scenario == 'normal')
        fixture.fail_install = scenario == 'install-failure'
        fixture.fail_probe = scenario == 'probe-failure'
        fixture.stuck = scenario == 'old-stuck'

        @contextlib.contextmanager
        def stage(**kwargs):
            path = case / 'stage'
            path.mkdir(exist_ok=True)
            yield str(path)

        output = io.StringIO()
        clock = [100.0]

        def monotonic():
            clock[0] += 0.01
            return clock[0]

        def sleep(seconds):
            clock[0] += seconds

        verifier = (
            (lambda *a, **k: (_ for _ in ()).throw(m.ControlError('not ready')))
            if scenario == 'verify-failure'
            else None
        )
        with (
            fixture.context(verifier=verifier),
            patch.object(m, 'datetime', FrozenDateTime),
            patch.object(m.time, 'monotonic', side_effect=monotonic),
            patch.object(m.time, 'sleep', side_effect=sleep),
            patch.object(m.time, 'time', return_value=1000.0),
            patch.object(m.tempfile, 'TemporaryDirectory', side_effect=stage),
            contextlib.redirect_stdout(output),
        ):
            try:
                value = str(fixture.activate())
                if scenario == 'repeat':
                    value = str(fixture.activate())
                outcome = ['return', value]
            except Exception as exc:
                outcome = [
                    type(exc).__name__,
                    str(exc),
                    type(exc.__cause__).__name__,
                    str(exc.__cause__),
                ]
        assert (outcome[0] == 'return') == (scenario in ('normal', 'repeat')), (scenario, outcome)
        files = {
            str(path.relative_to(case)): hashlib.sha256(path.read_bytes()).hexdigest()
            for path in sorted(case.rglob('*'))
            if path.is_file()
        }
        results.append(
            dict(
                scenario=scenario,
                calls=fixture.calls,
                outcome=outcome,
                info=fixture.info,
                files=files,
                stdout=output.getvalue(),
            )
        )
    return results


def standby_trace(root, folder):
    """以假进程/假传输模拟退出、二轮恢复和旧进程等待，记录严格有序副作用。"""
    import standby as m

    results = []
    for scenario in ['normal', 'retained', 'reused', 'retry', 'old-owned', 'preflight-failure']:
        case = folder / scenario
        case.mkdir()
        events = []
        attempt = [0]

        class Stop:
            value = False

            def is_set(self):
                return self.value

            def wait(self, seconds):
                events.append(['wait', seconds])
                if scenario == 'preflight-failure':
                    self.value = True
                return self.value

        stopped = Stop()

        class Writer:
            def __init__(self, *args):
                events.append(['writer-new'])

            def submit(self, row):
                events.append(['status', row])

            def message(self, text, *args):
                events.append(['message', text % args if args else text])

            def close(self):
                events.append(['writer-close'])

        class Node:
            pid = 42
            stdout = 'pipe'
            returncode = None

            def poll(self):
                events.append(['node-poll', self.returncode])
                return self.returncode

        node = Node()

        class Agent:
            pid = 55

            def wait(self, timeout):
                events.append(['agent-wait', timeout])

        agent = Agent()

        class Transport:
            def __init__(self, *args):
                events.append(['transport-new', args[1] is not None, args[2] is not None])

            def start(self):
                events.append(['transport-start'])

            def snapshot(self):
                return {'agent_pid': 55, 'agent_connected': True}

            def close(self):
                events.append(['transport-close'])
                return None if scenario == 'reused' else agent

        class Pump:
            def __init__(self, *args):
                events.append(['pump-new'])

            def join(self, timeout):
                events.append(['pump-join', timeout])

        class Sock:
            def close(self):
                events.append(['socket-close'])

        channel = types.SimpleNamespace(sock=Sock())

        class StopNode:
            def __init__(self, obj):
                self.obj = obj

            def request(self):
                events.append(['native-stop'])
                self.obj.returncode = 0

        class Handler:
            def close(self):
                events.append(['handler-close'])

        def logger(path, **kwargs):
            events.append(['logger', str(path)])
            return types.SimpleNamespace(handlers=[Handler()])

        def preflight(*args, **kwargs):
            events.append(['preflight'])
            if scenario == 'preflight-failure':
                raise m.ControlError('missing UART')
            return case / 'binary', agent if scenario == 'reused' else None

        def popen(argv, **kwargs):
            events.append(['spawn', argv])
            return node if '--ros-args' in argv else agent

        def monitor(channel, node, stopped, observe, **kwargs):
            events.append(['monitor'])
            attempt[0] += 1
            observe(
                {
                    'status': 'CONNECTED',
                    'reason': 'feedback',
                    'state': {'armed': False, 'landed': True},
                    'state_age_seconds': 0.25,
                    'controller_pid': 42,
                    'feedback_current': True,
                }
            )
            if scenario == 'retry' and attempt[0] == 1:
                raise RuntimeError('fixture IPC failure')
            stopped.value = True
            return {'armed': False, 'landed': True}, 99.5, None

        def previous(*args):
            events.append(['previous-controller-check'])
            if scenario == 'old-owned':
                stopped.value = True
                return False
            return True

        def ipc(path):
            target = case / 'ipc/ctl.sock'
            target.parent.mkdir(exist_ok=True)
            return target

        def finish(*args):
            events.append(['finish-agent', args[1:]])
            return scenario != 'retained'

        with (
            patch.object(m.Path, 'home', return_value=case),
            patch.object(m.os, 'getpid', return_value=41),
            patch.object(m.time, 'monotonic', return_value=100.0),
            patch.object(
                m.time, 'sleep', side_effect=lambda seconds: events.append(['sleep', seconds])
            ),
            patch.object(m, 'StatusWriter', Writer),
            patch.object(m, 'rotating_logger', side_effect=logger),
            patch.object(m, 'wait_for_previous_controller', side_effect=previous),
            patch.object(m, 'preflight', side_effect=preflight),
            patch.object(
                m,
                'validate',
                return_value={
                    'fc01_system': {'ros__parameters': {'flight_actions_enabled': False}}
                },
            ),
            patch.object(m, 'make_socket_path', side_effect=ipc),
            patch.object(m.subprocess, 'Popen', side_effect=popen),
            patch.object(m, 'AgentTransport', Transport),
            patch.object(m, 'LogPump', Pump),
            patch.object(
                m,
                'connect_controller',
                side_effect=lambda *args: events.append(['connect']) or channel,
            ),
            patch.object(m, 'monitor', side_effect=monitor),
            patch.object(m, 'NodeStop', StopNode),
            patch.object(
                m,
                'controller_result',
                side_effect=lambda node: {'controller_exit_reason': 'exit=0'},
            ),
            patch.object(m, 'finish_agent', side_effect=finish),
            patch.object(m.fcntl, 'flock', side_effect=lambda *args: events.append(['lock'])),
        ):
            m.supervise(
                case / 'sdk',
                case / 'agent',
                case / 'runtime',
                stopped,
                status_metadata={'entry_version': 'fixed'},
            )
        results.append(dict(scenario=scenario, events=events))
    return results


def launcher_trace(root, folder):
    """比较控制台读写顺序与退出；Mission/连接/进程全部为内存对象，不会发布飞行动作。"""
    import launcher as m

    results = []
    for scenario in ['input', 'check', 'ipc-failure']:
        events = []
        frames = (
            ['takeoff 1.5\n', ' \n', 'status\n', 'takeoff bad\n', 'land\n', 'quit\n']
            if scenario == 'input'
            else ['status\n']
        )
        frame = [0]

        class Node:
            returncode = 0

            def poll(self):
                return None if frame[0] < len(frames) else 0

        class Display:
            def __init__(self, *args):
                events.append(['display-new'])

            def say(self, message):
                events.append(['say', message])

            def close(self):
                events.append(['display-close'])

        class Evidence:
            last_state = {'phase': 'IDLE'}
            last_state_at = 100.0

            def command(self, *args):
                events.append(['command', *args])

            def failure(self, error):
                events.append(['failure', str(error)])

            def checkpoint(self, text):
                events.append(['checkpoint', text])

            def record(self, obj, now):
                events.append(['record', obj, now])
                self.last_state = obj

        class Mission:
            submitted = False
            pending = None
            error = None
            stage = ''
            target = 0

            def __init__(self, height):
                events.append(['mission-new', height])

            def request_takeoff(self, height, now):
                events.append(['takeoff-request', height, now])
                self.target = height
                self.stage = 'ISSUED_TAKEOFF'

            def cancel(self):
                events.append(['mission-cancel'])
                self.stage = ''

            def fail(self, error):
                events.append(['mission-fail', error])

            def observe(self, *args):
                events.append(['mission-observe', *args])

            def reply(self, *args):
                events.append(['mission-reply', *args])

            def poll(self, now):
                events.append(['mission-poll', now])
                return ('TAKEOFF', [self.target]) if self.stage == 'ISSUED_TAKEOFF' else None

            def sent(self, seq, op):
                events.append(['mission-sent', seq, op])
                self.stage = ''

        class Sock:
            def close(self):
                events.append(['socket-close'])

        class Channel:
            sock = Sock()
            read_error = None
            seq = 0

            def send(self, op, values):
                self.seq += 1
                events.append(['send', self.seq, op, values])
                return self.seq

            def read(self):
                events.append(['read'])
                if scenario == 'ipc-failure':
                    raise m.ControlError('fixture IPC lost')
                return [{'kind': 'state', 'phase': 'IDLE', 'armed': False}]

        class Stdin:
            def readline(self):
                return frames[frame[0] - 1]

        stdin = Stdin()
        channel = Channel()

        class Stop:
            def __init__(self, node):
                pass

            def request(self):
                events.append(['native-stop'])

        def selected(readers, *args):
            frame[0] += 1
            return readers, [], []

        with (
            patch.object(m, 'Display', Display),
            patch.object(m, 'Mission', Mission),
            patch.object(m, 'NodeStop', Stop),
            patch.object(m.time, 'monotonic', return_value=100.0),
            patch.object(m.select, 'select', side_effect=selected),
            patch.object(m.sys, 'stdin', stdin),
            patch.object(m, 'readiness_issues', return_value=['fixture condition']),
            patch.object(m.time, 'sleep', side_effect=lambda seconds: None),
        ):
            value = m.run_console(
                channel,
                Node(),
                Evidence(),
                {'aa_max_height': 1.5},
                scenario != 'check',
                0.0,
                [False],
            )
        results.append(dict(scenario=scenario, result=value, events=events))
    return results


def launcher_entry_trace(root, folder):
    """比较legacy入口的参数门、配置、资源归属和收尾，所有进程及控制循环均mock。"""
    import launcher as m

    results = []
    for scenario in ['check', 'flight', 'reused', 'flight-no-tty', 'bad-duration']:
        case = folder / scenario
        case.mkdir(mode=0o700)
        sdk = case / 'sdk'
        (sdk / 'config').mkdir(parents=True)
        shutil.copyfile(
            root / 'native/config/attitude_altitude.yaml', sdk / 'config/attitude_altitude.yaml'
        )
        binary = case / 'binary'
        binary.write_bytes(b'fixture-native-binary')
        events = []

        class Display:
            def __init__(self, *args):
                events.append(['display-new'])

            def say(self, message):
                events.append(['say', message])

            def close(self):
                events.append(['display-close'])

        class Evidence:
            last_state = {'armed': False}
            last_state_at = 100.0

            def __init__(self, path, metadata):
                self.folder = path
                path.mkdir(mode=0o700, parents=True)
                self.events = types.SimpleNamespace(close=lambda: events.append(['events-close']))
                events.append(['evidence-new', metadata])

            def command(self, *args):
                events.append(['command', *args])

            def failure(self, error):
                events.append(['failure', str(error)])

            def checkpoint(self, text):
                events.append(['checkpoint', text])

            def write_json(self, name, row):
                events.append(['write-json', name, row])

            def report(self, outcome):
                events.append(['report', outcome])
                return self.folder / 'report.zip'

        class Process:
            pid = 42

            def poll(self):
                return 0

        agent = types.SimpleNamespace(pid=55)
        channel = types.SimpleNamespace(
            sock=types.SimpleNamespace(close=lambda: events.append(['socket-close']))
        )

        def popen(argv, **kwargs):
            events.append(['spawn', argv])
            return Process() if '--ros-args' in argv else agent

        def run_console(channel, node, evidence, params, flight, seconds, stopped):
            events.append(['console', params, flight, seconds])
            return 'operator_quit' if flight else 'check_pass'

        argv = [
            '--root',
            str(sdk),
            '--agent',
            str(case / 'agent'),
            '--check-seconds',
            '1' if scenario == 'bad-duration' else '10',
        ]
        if scenario in ('flight', 'flight-no-tty'):
            argv.append('--flight')
        stderr = io.StringIO()
        with (
            patch.object(m.Path, 'home', return_value=case),
            patch.object(m, 'datetime', FrozenDateTime),
            patch.object(m.uuid, 'uuid4', return_value=types.SimpleNamespace(hex='a' * 32)),
            patch.object(m, 'Display', Display),
            patch.object(m, 'Evidence', Evidence),
            patch.object(
                m, 'preflight', return_value=(binary, agent if scenario == 'reused' else None)
            ),
            patch.object(m, 'make_socket_path', return_value=case / 'ipc.sock'),
            patch.object(m, 'connect_controller', return_value=channel),
            patch.object(m, 'run_console', side_effect=run_console),
            patch.object(m.subprocess, 'Popen', side_effect=popen),
            patch.object(
                m.signal,
                'signal',
                side_effect=lambda sig, handler: events.append(['signal', int(sig)]),
            ),
            patch.object(m.sys.stdin, 'isatty', return_value=scenario != 'flight-no-tty'),
            patch.object(m.time, 'monotonic', return_value=100.0),
            patch.object(
                m,
                'finish_agent',
                side_effect=lambda *args: events.append(['finish-agent', list(args[1:])]) or True,
            ),
            contextlib.redirect_stderr(stderr),
        ):
            try:
                value = ['return', m.main(argv)]
            except SystemExit as exc:
                value = ['SystemExit', exc.code]
        configs = {
            str(path.relative_to(case)): path.read_text() for path in case.rglob('runtime.yaml')
        }
        results.append(
            dict(
                scenario=scenario,
                result=value,
                events=events,
                stderr=stderr.getvalue(),
                configs=configs,
            )
        )
    return results


def fcctl_trace(root, folder):
    """只在假socket记录线协议，覆盖单次ACK/状态确认、部分消息及台架二次确认；不建立真实连接。"""
    sys.path.insert(1, str(root / 'native/tools'))
    import fcctl as m

    results = []
    for scenario in [
        'status',
        'mode',
        'attitude-ready',
        'reject',
        'error',
        'truncated',
        'interactive',
    ]:
        events = []
        clock = [100.0]
        iteration = [0]
        state = dict(
            kind='state',
            phase='OFFBOARD_READY',
            nav_state=14,
            armed=False,
            ground_test='PASS',
            estimate_valid=True,
            reason='fixture',
        )
        if scenario == 'interactive':
            state.update(
                profile='attitude_altitude',
                bench_arm_only_enabled=True,
                landed=True,
                at_rest=True,
                status_fresh=True,
                local_fresh=True,
                land_fresh=True,
                failsafe_fresh=True,
                battery_fresh=True,
                source_clock_valid=True,
                source_clock_validated=True,
                attitude_fresh=True,
                z_valid=True,
                vz_valid=True,
                preflight=True,
                battery_connected=True,
                authority_mode='RC_TAKEOVER',
                authority_ready=True,
                authority_attempt_mode='RC_TAKEOVER',
                authority_locked=True,
                rc_link_ready=True,
                rc_switches_ready=True,
                rc_kill_latched=False,
                flight_actions_enabled=True,
                arm_enabled=True,
                takeoff_enabled=False,
                flight_parameters_confirmed=True,
                publication_fault_latched=False,
                authority_ground_failed_mask=0,
                stationary_seconds=3.0,
                required_ground_seconds=2.0,
                ground_vz_ned=0.0,
                ground_vz_tolerance=0.1,
            )
        lines = ['arm-only\n', 'BENCH ARM CONFIRM\n', 'arm-only\n', 'help\n', 'quit\n']

        class Stdin:
            def isatty(self):
                return True

            def readline(self):
                return lines[min(iteration[0] - 1, len(lines) - 1)]

        stdin = Stdin()

        class Sock:
            def settimeout(self, value):
                events.append(['timeout', value])

            def connect(self, path):
                events.append(['connect', path])

            def setblocking(self, value):
                events.append(['blocking', value])

            def __enter__(self):
                return self

            def __exit__(self, *args):
                events.append(['close'])

            def send(self, packet):
                events.append(['wire', packet.decode()])
                return len(packet)

            def recv(self, n):
                if scenario == 'interactive':
                    return (json.dumps(state) + '\n').encode()
                if scenario == 'error':
                    return b'{"kind":"error","message":"fixture"}\n'
                if scenario == 'truncated':
                    return b''
                reply = {
                    'kind': 'reply',
                    'seq': 1,
                    'accepted': scenario != 'reject',
                    'message': 'fixture',
                }
                return (
                    json.dumps(reply)
                    + '\n'
                    + (json.dumps(state) + '\n' if scenario in ('mode', 'attitude-ready') else '')
                ).encode()

        sock = Sock()

        def selected(readers, writers, *args):
            if writers:
                return [], writers, []
            iteration[0] += 1
            return readers, [], []

        def now():
            clock[0] += 0.01
            return clock[0]

        args = types.SimpleNamespace(
            command=(
                None
                if scenario == 'interactive'
                else scenario if scenario in ('mode', 'attitude-ready') else 'status'
            ),
            socket='/fixture/socket',
            timeout=3.0,
            json=False,
            bench_arm_only=True,
        )
        out = io.StringIO()
        err = io.StringIO()
        with (
            patch.object(m.socket, 'socket', return_value=sock),
            patch.object(m.select, 'select', side_effect=selected),
            patch.object(m.time, 'monotonic', side_effect=now),
            patch.object(m.sys, 'stdin', stdin),
            contextlib.redirect_stdout(out),
            contextlib.redirect_stderr(err),
        ):
            try:
                value = ['return', m.run(args)]
            except Exception as exc:
                value = [type(exc).__name__, str(exc)]
        expected = (
            ['ConnectionError', '控制节点已断开；检查机载日志']
            if scenario == 'truncated'
            else ['return', 2 if scenario in ('reject', 'error') else 0]
        )
        assert value == expected, (scenario, value)
        if scenario == 'interactive':
            arm_wires = [
                event for event in events if event[0] == 'wire' and ' ARM_ONLY' in event[1]
            ]
            assert len(arm_wires) == 1, arm_wires
            assert not any(event[0] == 'wire' and ' TAKEOFF' in event[1] for event in events)
        results.append(
            dict(
                scenario=scenario,
                events=events,
                result=value,
                stdout=out.getvalue(),
                stderr=err.getvalue(),
            )
        )
    return results


def probe_trace(root, folder, *, expect_cleanup_gap=True):
    """用假的ROS上下文/订阅调度比较字段抽取、地面证据、RC来源与finally释放顺序。"""
    import navigation_probe as m

    results = []
    for scenario in ['ground', 'rc', 'armed', 'existing-required', 'disk-full']:
        case = folder / scenario
        case.mkdir()
        events = []
        tick = [0]
        mono = [100.0]

        class Msg:
            pass

        types_by_name = {
            name: type(name, (), {})
            for name in [
                'VehicleStatus',
                'VehicleLandDetected',
                'ManualControlSetpoint',
                'ManualControlSwitches',
                'FailsafeFlags',
            ]
        }

        class Node:
            def __init__(self):
                self.callbacks = []

            def create_subscription(self, cls, topic, callback, qos):
                events.append(['subscribe', cls.__name__, topic, qos])
                self.callbacks.append((cls, callback))
                return len(self.callbacks)

            def count_publishers(self, topic):
                return 1

            def destroy_node(self):
                events.append(['destroy-node'])

        node = Node()
        alive = [False]

        def init():
            alive[0] = True
            events.append(['init'])

        def shutdown():
            alive[0] = False
            events.append(['shutdown'])

        def spin(node, timeout_sec):
            tick[0] += 1
            mono[0] += 0.2
            for cls, callback in node.callbacks:
                msg = types.SimpleNamespace(
                    timestamp=int((1000 + tick[0] * 0.2) * 1e6),
                    timestamp_sample=int((1000 + tick[0] * 0.2) * 1e6),
                    system_id=1,
                    component_id=1,
                    arming_state=2 if scenario == 'armed' else 1,
                    landed=True,
                    at_rest=True,
                    valid=True,
                    data_source=1,
                    sticks_moving=False,
                    roll=0.0,
                    pitch=0.0,
                    yaw=0.0,
                    throttle=-1.0,
                    arm_switch=3,
                    kill_switch=3,
                    mode_slot=4,
                    return_switch=3,
                    loiter_switch=3,
                    offboard_switch=3,
                    manual_control_signal_lost=False,
                )
                callback(msg)

        ros = types.ModuleType('rclpy')
        ros.init = init
        ros.create_node = lambda name: events.append(['node', name]) or node
        ros.spin_once = spin
        ros.ok = lambda: alive[0]
        ros.shutdown = shutdown
        qos = types.ModuleType('rclpy.qos')
        qos.qos_profile_sensor_data = 'sensor-qos'
        msgs = types.ModuleType('px4_msgs.msg')
        for name, cls in types_by_name.items():
            setattr(msgs, name, cls)
        output = io.StringIO()
        path = case / 'probe.json'
        original_write_text = Path.write_text

        def save_probe(path_to_write, data, *args, **kwargs):
            if scenario == 'disk-full' and path_to_write == path:
                raise OSError('fixture disk full')
            return original_write_text(path_to_write, data, *args, **kwargs)

        with (
            patch.object(m.Path, 'write_text', autospec=True, side_effect=save_probe),
            patch.dict(
                sys.modules,
                {
                    'rclpy': ros,
                    'rclpy.qos': qos,
                    'px4_msgs': types.ModuleType('px4_msgs'),
                    'px4_msgs.msg': msgs,
                },
            ),
            patch.object(
                m,
                'choose_agent',
                return_value=(
                    None if scenario == 'existing-required' else types.SimpleNamespace(pid=55)
                ),
            ),
            patch.object(m, 'process_list', return_value=[]),
            patch.object(m, 'uart_owners', return_value=[]),
            patch.object(m.time, 'time', side_effect=lambda: 1000 + (mono[0] - 100)),
            patch.object(m.time, 'monotonic', side_effect=lambda: mono[0]),
            contextlib.redirect_stdout(output),
        ):
            try:
                value = [
                    'return',
                    m.probe(
                        case / 'sdk',
                        case / 'agent',
                        path,
                        require_existing_agent=True,
                        require_rc_safe=scenario == 'rc',
                    ),
                ]
            except Exception as exc:
                value = [type(exc).__name__, str(exc)]
        assert (value[0] == 'return') == (scenario in ('ground', 'rc')), (scenario, value)
        if scenario == 'disk-full':
            assert value == ['OSError', 'fixture disk full']
            if expect_cleanup_gap:
                assert alive[0] and ['destroy-node'] not in events and ['shutdown'] not in events
            else:
                assert not alive[0] and ['destroy-node'] in events and ['shutdown'] in events
        results.append(
            dict(
                scenario=scenario,
                result=value,
                events=events,
                stdout=output.getvalue(),
                evidence=json.loads(path.read_text()) if path.exists() else None,
                ros_context_alive=alive[0],
                known_cleanup_defect=scenario == 'disk-full' and alive[0],
            )
        )
    return results


def main():
    """只由测试驱动调用；目标源码树来自基准Git对象或当前工作区，不读取用户服务。"""
    parser = argparse.ArgumentParser()
    parser.add_argument('root', type=Path)
    parser.add_argument('folder', type=Path)
    parser.add_argument('scenario')
    parser.add_argument('--probe-cleanup-fixed', action='store_true')
    args = parser.parse_args()
    root = args.root.resolve()
    sys.path.insert(0, str(root))
    # 只清理测试驱动明确拥有的fixture目录，拒绝CLI误传源码/home等用户目录。
    marker = args.folder.parent / '.fc01-stage2-python'
    if (
        args.folder.name != 'fixture'
        or args.folder.is_symlink()
        or marker.is_symlink()
        or not marker.is_file()
        or marker.read_text() != '5198d33f25aaaf2abc7e4b078169ae84d5c4b2c4\n'
    ):
        parser.error('fixture目录缺少第二阶段测试所有权标记')
    if args.folder.exists():
        shutil.rmtree(args.folder)
    args.folder.mkdir(parents=True)
    kwargs = (
        {'expect_cleanup_gap': not args.probe_cleanup_fixed} if args.scenario == 'probe' else {}
    )
    trace = globals()[args.scenario + '_trace'](root, args.folder, **kwargs)

    def canonical(value):
        if isinstance(value, float) and not math.isfinite(value):
            return {'nonfinite': repr(value)}
        if isinstance(value, dict):
            return {k: canonical(v) for k, v in value.items()}
        if isinstance(value, (list, tuple)):
            return [canonical(v) for v in value]
        return value

    print(json.dumps(canonical(trace), ensure_ascii=False, sort_keys=True, allow_nan=False))


if __name__ == '__main__':
    main()
