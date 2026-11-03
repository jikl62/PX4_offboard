import importlib
import contextlib
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

class StandbyTests(unittest.TestCase):
    def module(self):
        self.assertTrue((ROOT/'standby.py').exists(), 'continuous closed-gate standby is missing')
        return importlib.import_module('standby')

    def peer(self, mode='normal'):
        folder = tempfile.TemporaryDirectory(prefix='fc01_standby_test_')
        self.addCleanup(folder.cleanup)
        trace = Path(folder.name)/'trace'
        a, b = socket.socketpair()
        self.addCleanup(a.close)
        node = subprocess.Popen([sys.executable, str(ROOT/'tests/standby_peer.py'),
                                 str(b.fileno()), str(trace), mode], pass_fds=(b.fileno(),))
        b.close()
        def cleanup():
            if node.poll() is None:
                node.terminate()
            node.wait(timeout=3)
        self.addCleanup(cleanup)
        return a, node, trace

    def test_standby_runs_without_stdin_until_signal_and_only_requests_status(self):
        m = self.module()
        a, node, trace = self.peer()
        stopped = threading.Event()
        timer = threading.Timer(.45, stopped.set)
        timer.start()
        self.addCleanup(timer.cancel)
        states = []
        start = time.monotonic()
        state, at, error = m.monitor(m.Channel(a), node, stopped, states.append)
        self.assertGreater(time.monotonic()-start, .4)
        self.assertEqual(node.returncode, 0)
        self.assertIsNone(error)
        self.assertFalse(state['armed'])
        self.assertGreater(at, start)
        lines = trace.read_text().splitlines()
        self.assertEqual(lines[-1], 'STOPPED')
        self.assertEqual([line.split()[1] for line in lines[:-1]], ['STATUS'])
        self.assertTrue(any(row['status'] == 'CONNECTED' for row in states))

    def test_socket_failure_waits_for_live_native_exit(self):
        m = self.module()
        a, node, trace = self.peer('close')
        state, at, error = m.monitor(m.Channel(a), node, threading.Event(), lambda _: None)
        self.assertIsNotNone(error)
        self.assertEqual(node.returncode, 0)
        self.assertIn('STOPPED', trace.read_text())

    def test_no_fresh_state_is_waiting_even_while_native_is_alive(self):
        m = self.module()
        a, node, trace = self.peer('silent')
        stopped = threading.Event()
        timer = threading.Timer(.2, stopped.set)
        timer.start()
        self.addCleanup(timer.cancel)
        states = []
        m.monitor(m.Channel(a), node, stopped, states.append)
        self.assertFalse(any(x['status'] == 'CONNECTED' for x in states))
        self.assertTrue(any(x['status'] == 'WAITING' for x in states))

    def test_steady_feedback_refreshes_snapshot_before_view_freshness_expires(self):
        m = self.module()
        a, node, _ = self.peer()
        stopped = threading.Event()
        timer = threading.Timer(2.2, stopped.set)
        timer.start()
        self.addCleanup(timer.cancel)
        rows = []
        m.monitor(m.Channel(a), node, stopped,
                  lambda row: rows.append((time.monotonic(), row)))
        current = [at for at, row in rows if row['status'] == 'CONNECTED'
                   and row.get('feedback_current') is True]
        self.assertGreaterEqual(len(current), 2, 'steady live feedback needs periodic current snapshots')
        self.assertLessEqual(max(b-a for a, b in zip(current, current[1:])), 1.5)

    def test_native_log_pump_rotates_real_subprocess_output(self):
        m = self.module()
        with tempfile.TemporaryDirectory() as d:
            path = Path(d)/'controller.log'
            log = m.rotating_logger(path, max_bytes=16384)
            child = subprocess.Popen([sys.executable, '-c',
                'import sys; sys.stdout.write("a"*100000+"LAST_RECORD\\n")'], stdout=subprocess.PIPE)
            pump = m.LogPump(child.stdout, log)
            child.wait(timeout=4)
            pump.join(timeout=4)
            self.assertFalse(pump.is_alive())
            files = list(Path(d).glob('controller.log*'))
            self.assertEqual(len(files), 3)
            self.assertLess(sum(x.stat().st_size for x in files), 3*16384+8192)
            self.assertIn('LAST_RECORD', path.read_text())
            for handler in log.handlers:
                handler.close()

    def test_latest_status_is_atomic_and_does_not_accumulate_files(self):
        m = self.module()
        with tempfile.TemporaryDirectory() as d:
            path = Path(d)
            for i in range(40):
                m.write_status(path, {'status': 'WAITING', 'attempt': i})
                self.assertEqual(json.loads((path/'status.json').read_text())['attempt'], i)
            self.assertEqual([p.name for p in path.iterdir()], ['status.json'])

    def test_boot_config_keeps_all_action_gates_closed(self):
        m = self.module()
        import yaml
        base = yaml.safe_load((ROOT/'tests/fixtures/base.yaml').read_text())
        config = m.standby_config(base)
        from fc01_control import ACTION_FLAGS
        self.assertTrue(all(config['fc01_system']['ros__parameters'][x] is False for x in ACTION_FLAGS))
        self.assertTrue(config['fc01_system']['ros__parameters']['aa_vertical_feedback_guard_enabled'])

    def test_agent_loss_stops_native_before_recovery_attempt(self):
        m = self.module()
        a, node, trace = self.peer()
        start = time.monotonic()
        health = lambda: 'Agent exited' if time.monotonic()-start > .1 else None
        state, at, error = m.monitor(m.Channel(a), node, threading.Event(), lambda _: None, health)
        self.assertEqual(error, 'Agent exited')
        self.assertEqual(node.returncode, 0)
        self.assertIn('STOPPED', trace.read_text())

    def test_missing_device_retries_after_30_seconds_without_new_session_folders(self):
        m = self.module()
        manager = importlib.import_module('autostart')
        elapsed = [0]
        midway = io.StringIO()
        class StopOnWait(threading.Event):
            def __init__(self):
                super().__init__()
                self.waits = []
            def wait(self, timeout=None):
                self.waits.append(timeout)
                elapsed[0] += timeout
                if elapsed[0] == 15:
                    end=time.monotonic()+1
                    while not (home/'FC01_autostart/status.json').exists() and time.monotonic()<end:
                        time.sleep(.002)
                    with contextlib.redirect_stdout(midway):
                        manager.status(home)
                if elapsed[0] >= 30:
                    self.set()
                return self.is_set()
        with tempfile.TemporaryDirectory() as d:
            home = Path(d)
            stopped = StopOnWait()
            with patch.object(Path, 'home', return_value=home), \
                 patch.object(m.time, 'time', side_effect=lambda: 1000+elapsed[0]), \
                 patch.object(manager.subprocess, 'run', return_value=subprocess.CompletedProcess([], 0, 'active\n', '')), \
                 patch.object(m, 'preflight', side_effect=m.ControlError('missing UART')) as check:
                m.supervise(home/'project', home/'agent', home/'FC01_autostart', stopped)
            self.assertEqual(sum(stopped.waits), 30)
            self.assertEqual(check.call_count, 1)
            self.assertIn('后台：WAITING', midway.getvalue())
            self.assertIn('missing UART', midway.getvalue())
            self.assertNotIn('STALE', midway.getvalue())
            self.assertIn('missing UART', (home/'FC01_autostart/standby.log').read_text())
            self.assertEqual(json.loads((home/'FC01_autostart/status.json').read_text())['status'], 'STOPPED')
            self.assertFalse((home/'FC01_control_runs').exists())

    def test_supervisor_respects_existing_operator_lock(self):
        m = self.module()
        import fcntl
        class StopOnWait(threading.Event):
            def wait(self, timeout=None):
                self.set()
                return True
        with tempfile.TemporaryDirectory() as d:
            home = Path(d)
            cache = home/'.cache/fc01_operator'
            cache.mkdir(parents=True, mode=0o700)
            with open(cache/'launcher.lock', 'w') as lock:
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
                with patch.object(Path, 'home', return_value=home), patch.object(m, 'preflight', side_effect=AssertionError('must not start beside operator')):
                    m.supervise(home/'project', home/'agent', home/'state', StopOnWait())
            self.assertIn('旧入口正在运行', (home/'state/standby.log').read_text())

    def test_full_supervisor_reuses_agent_and_launches_closed_gate_native(self):
        m = self.module()
        import yaml
        from launcher import Process
        with tempfile.TemporaryDirectory() as d:
            home = Path(d)
            binary = home/'native_fixture'
            binary.write_text((ROOT/'tests/standby_native.py').read_text())
            binary.chmod(0o700)
            agent = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(30)'])
            def stop_agent():
                if agent.poll() is None:
                    agent.terminate()
                agent.wait(timeout=3)
            self.addCleanup(stop_agent)
            existing = Process(agent.pid, os.getuid(), tuple(agent.args))
            base = yaml.safe_load((ROOT/'tests/fixtures/base.yaml').read_text())
            stopped = threading.Event()
            timer = threading.Timer(.8, stopped.set)
            timer.start()
            self.addCleanup(timer.cancel)
            a, b = socket.socketpair()
            self.addCleanup(a.close)
            self.addCleanup(b.close)
            real_popen = subprocess.Popen
            real_read_bytes = Path.read_bytes
            def proc_bytes(path):
                # Host /proc PID namespace differs from subprocess handles here.
                if path == Path('/proc', str(agent.pid), 'cmdline'):
                    return b'\0'.join(os.fsencode(x) for x in agent.args)+b'\0'
                return real_read_bytes(path)
            def launch(argv, **kwargs):
                return real_popen([*argv, '--test-fd='+str(b.fileno())],
                                  **kwargs, pass_fds=(b.fileno(),))
            with patch.object(Path, 'home', return_value=home), \
                 patch.object(m, 'preflight', return_value=(binary, existing)), \
                 patch.object(m, 'validate', return_value=m.standby_config(base)), \
                 patch.object(m.subprocess, 'Popen', side_effect=launch), \
                 patch.object(Path, 'read_bytes', new=proc_bytes), \
                 patch.object(m, 'connect_controller', return_value=m.Channel(a)):
                m.supervise(home/'project', home/'agent', home/'state', stopped)
            self.assertIsNone(agent.poll(), 'a reused Agent must remain alive')
            agent.terminate()
            agent.wait(timeout=3)
            self.assertTrue((home/'state/native_trace.json').exists(), (home/'state/controller.log').read_text())
            trace = json.loads((home/'state/native_trace.json').read_text())
            self.assertEqual(trace['ops'], ['STATUS'])
            self.assertEqual(trace['csv_value'], '')
            self.assertTrue(trace['external_file_logs_disabled'])
            self.assertTrue(trace['all_gates_closed'])
            self.assertEqual(list((home/'.cache/fc01_operator/ipc').iterdir()), [])

    def test_standby_overrides_source_csv_file_path_with_yaml_empty_string(self):
        m = self.module()
        import yaml
        base = yaml.safe_load((ROOT/'tests/fixtures/base.yaml').read_text())
        base['fc01_system']['ros__parameters']['csv_path'] = '/tmp/source-unbounded.csv'
        serialized = yaml.safe_dump(m.standby_config(base))
        effective = yaml.safe_load(serialized)['fc01_system']['ros__parameters']['csv_path']
        self.assertIsInstance(effective, str)
        self.assertEqual(effective, '')

    def test_retained_owned_agent_is_reused_after_standby_stop_and_start(self):
        m = self.module()
        import yaml
        from launcher import Process
        with tempfile.TemporaryDirectory() as d:
            home = Path(d)
            binary, agent_path = home/'native_fixture', home/'agent'
            binary.write_text((ROOT/'tests/standby_native.py').read_text())
            binary.chmod(0o700)
            base = yaml.safe_load((ROOT/'tests/fixtures/base.yaml').read_text())
            agents = []
            real_popen, real_read = subprocess.Popen, Path.read_bytes
            def cleanup_agents():
                for p in agents:
                    if p.poll() is None:
                        p.terminate()
                    p.wait(timeout=3)
            self.addCleanup(cleanup_agents)
            def preflight(*_):
                existing = Process(agents[0].pid, os.getuid(), tuple(agents[0].args)) if agents else None
                return binary, existing
            def proc_bytes(path):
                if agents and path == Path('/proc', str(agents[0].pid), 'cmdline'):
                    return b'\0'.join(os.fsencode(x) for x in agents[0].args)+b'\0'
                return real_read(path)
            for cycle in range(2):
                a, b = socket.socketpair()
                stopped = threading.Event()
                timer = threading.Timer(.6, stopped.set)
                timer.start()
                def launch(argv, **kwargs):
                    if argv[0] == str(agent_path):
                        p = real_popen([sys.executable, '-c', 'import time; time.sleep(30)'], **kwargs)
                        agents.append(p)
                        return p
                    return real_popen([*argv, '--test-fd='+str(b.fileno()), '--test-no-ground'],
                                      **kwargs, pass_fds=(b.fileno(),))
                try:
                    with patch.object(Path, 'home', return_value=home), \
                         patch.object(m, 'preflight', side_effect=preflight), \
                         patch.object(m, 'validate', return_value=m.standby_config(base)), \
                         patch.object(m.subprocess, 'Popen', side_effect=launch), \
                         patch.object(Path, 'read_bytes', new=proc_bytes), \
                         patch.object(m, 'connect_controller', return_value=m.Channel(a)):
                        m.supervise(home/'project', agent_path, home/'state', stopped)
                    self.assertEqual(len(agents), 1)
                    self.assertIsNone(agents[0].poll())
                    trace = json.loads((home/'state/native_trace.json').read_text())
                    self.assertEqual(trace['ops'], ['STATUS'])
                finally:
                    timer.cancel()
                    a.close()
                    b.close()

    def test_surviving_native_blocks_start_even_without_inherited_lock(self):
        self.module()
        import launcher
        with tempfile.TemporaryDirectory() as d:
            root = Path(d)
            (root/'config').mkdir()
            (root/'config/attitude_altitude.yaml').write_text('unused')
            survivor = launcher.Process(12345, os.getuid(), ('/opt/fc01_system_node', '--ros-args'))
            with patch.object(launcher.platform, 'machine', return_value='aarch64'), \
                 patch.object(launcher, 'verify_binary'), \
                 patch.object(launcher, 'process_list', return_value=[survivor]), \
                 patch.dict(os.environ, ROS_DOMAIN_ID='0', ROS_LOCALHOST_ONLY='0'):
                with self.assertRaisesRegex(launcher.ControlError, '已有控制器运行'):
                    launcher.preflight(root, root/'agent', '/dev/null')

if __name__ == '__main__':
    unittest.main()
