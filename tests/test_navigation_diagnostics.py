"""Real IPC/process regressions for current feedback and retained history."""
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

import launcher
import navigation
import navigation_service
import standby


STATE = dict(kind='state', phase='HEIGHT_HOLD', reason='last native feedback',
             status_fresh=True, local_fresh=True, nav_state=14, armed=True,
             relative_height_m=.42)


class NavigationDiagnosticsTests(unittest.TestCase):
    def test_closed_native_action_gate_labels_preparation_as_diagnostic(self):
        for closed in ('flight_actions_enabled', 'takeoff_enabled'):
            with self.subTest(closed=closed):
                state=dict(STATE,phase='RC_READY_WAIT_ARM',armed=False,
                           reason='RC_READY_WAIT_CH5_MANUAL_ARM',flight_enabled=False,
                           arm_enabled=False,flight_actions_enabled=True,takeoff_enabled=True,
                           rc_navigation_enabled=True)
                state[closed]=False
                row=dict(status='CONNECTED',reason='transport okay',supervisor_pid=10,
                         updated_unix=1000,state=state,state_age_seconds=.1,feedback_current=True,
                         flight_actions_enabled=True,rc_navigation_enabled=True)
                described=navigation_service.describe_row(row)
                self.assertEqual(described['status'],'DIAGNOSTIC')
                self.assertIs(described['state'],state)
                self.assertEqual(state['phase'],'RC_READY_WAIT_ARM')
                self.assertEqual(state['reason'],'RC_READY_WAIT_CH5_MANUAL_ARM')
                # The view also protects against a previously written READY
                # label and optimistic supervisor role metadata.
                for displayed in (described,dict(row,status='READY_WAIT_ARM')):
                    view=navigation.render_status(displayed,dict(ActiveState='active',MainPID='10'),now=1000)
                    self.assertIn('[DIAGNOSTIC]',view)
                    self.assertIn(closed+'=false',view)
                    self.assertIn('内部阶段：RC_READY_WAIT_ARM',view)
                    self.assertIn('禁止起飞和导航',view)
                    self.assertNotIn('已准备：CH5 手动解锁',view)

    def test_program_arm_disabled_does_not_disable_valid_manual_rc_guidance(self):
        state=dict(STATE,phase='RC_READY_WAIT_ARM',armed=False,
                   reason='RC_READY_WAIT_CH5_MANUAL_ARM',flight_enabled=False,
                   arm_enabled=False,flight_actions_enabled=True,takeoff_enabled=True,
                   rc_navigation_enabled=True)
        row=dict(status='CONNECTED',reason='transport okay',supervisor_pid=10,updated_unix=1000,
                 state=state,state_age_seconds=.1,feedback_current=True)
        described=navigation_service.describe_row(row)
        self.assertEqual(described['status'],'READY_WAIT_ARM')
        view=navigation.render_status(described,dict(ActiveState='active',MainPID='10'),now=1000)
        self.assertIn('已准备：CH5 手动解锁',view)
        self.assertIn('arm_enabled=false（程序解锁）',view)
        self.assertNotIn('[DIAGNOSTIC]',view)

    def test_unknown_native_gates_cannot_prompt_manual_arming(self):
        state=dict(STATE,phase='RC_READY_WAIT_ARM',armed=False)
        row=dict(status='READY_WAIT_ARM',reason='ready',supervisor_pid=10,updated_unix=1000,
                 state=state,state_age_seconds=.1,feedback_current=True,
                 flight_actions_enabled=True,rc_navigation_enabled=True)
        view=navigation.render_status(row,dict(ActiveState='active',MainPID='10'),now=1000)
        self.assertIn('flight_actions_enabled=未知',view)
        self.assertIn('尚未确认飞行动作许可',view)
        self.assertNotIn('已准备：CH5 手动解锁',view)

    def test_diagnostic_gates_do_not_hide_fault_or_revive_historical_feedback(self):
        state=dict(STATE,phase='SAFETY',reason='CONTROL_LOOP_STALL',
                   flight_actions_enabled=False,takeoff_enabled=False)
        row=dict(status='CONNECTED',reason='transport okay',supervisor_pid=10,updated_unix=1000,
                 state=state,state_age_seconds=.1,feedback_current=True)
        fault=navigation_service.describe_row(row)
        self.assertEqual(fault['status'],'FAULT')
        self.assertIn('[FAULT]',navigation.render_status(fault,dict(ActiveState='active',MainPID='10'),now=1000))
        state['phase']='RC_READY_WAIT_ARM'
        historical=dict(row,status='WAITING',reason='native disconnected',feedback_current=False)
        self.assertEqual(navigation_service.describe_row(historical)['status'],'WAITING')
        view=navigation.render_status(historical,dict(ActiveState='active',MainPID='10'),now=1000)
        self.assertIn('[WAITING]',view)
        self.assertIn('最近动作门（历史）',view)
        self.assertNotIn('[DIAGNOSTIC]',view)
        self.assertNotIn('已准备：CH5 手动解锁',view)

    def test_explicit_unconfirmed_liftoff_never_displays_height_hold_as_ready(self):
        state=dict(STATE,takeoff_stage='LIFTOFF_UNCONFIRMED',liftoff_confirmed=False,
                   target_height_settled=False,navigation_allowed=False,
                   liftoff_unconfirmed_reason='WAIT_NEW_LOCAL_MOTION_SAMPLES')
        row=dict(status='HEIGHT_HOLD',reason='previous phase',supervisor_pid=10,
                 updated_unix=1000,state=state,state_age_seconds=.1,feedback_current=True)
        view=navigation.render_status(row,dict(ActiveState='active',MainPID='10'),now=1000)
        self.assertIn('离地未确认',view)
        self.assertIn('WAIT_NEW_LOCAL_MOTION_SAMPLES',view)
        self.assertIn('导航未允许',view)
        self.assertNotIn('正在定高',view)

    def test_view_distinguishes_local_thrust_commit_from_px4_adoption(self):
        state=dict(STATE,takeoff_stage='TARGET_SETTLED',liftoff_confirmed=True,
                   target_height_settled=True,navigation_allowed=True,
                   target_local_commits=25,last_target_local_thrust_z=-.50,
                   px4_thrust_adoption='UNOBSERVED')
        row=dict(status='HEIGHT_HOLD',reason='hold',supervisor_pid=10,updated_unix=1000,
                 state=state,state_age_seconds=.1,feedback_current=True)
        view=navigation.render_status(row,dict(ActiveState='active',MainPID='10'),now=1000)
        self.assertIn('本地推力',view)
        self.assertIn('-0.5',view)
        self.assertIn('PX4 采用尚未取证',view)

    def peer(self, *, tail=b'', exit_code=0, exit_signal=None, silent_seconds=.08,
             keep_open=False, exit_on_stop=False):
        a, b = socket.socketpair()
        self.addCleanup(a.close)
        code = 'import os,signal,socket,sys,time;'
        code += ('signal.signal(signal.SIGTERM,lambda *_:sys.exit(0));' if exit_on_stop else
                 'signal.signal(signal.SIGTERM,lambda *_:None);')
        code += ('s=socket.socket(fileno=int(sys.argv[1]));'
                's.sendall(bytes.fromhex(sys.argv[2]));')
        if exit_signal is not None:
            code += 'os.kill(os.getpid(),int(sys.argv[3]));'
        else:
            if not keep_open and not silent_seconds > 3:
                code += 's.shutdown(socket.SHUT_WR);'
            code += 'time.sleep(float(sys.argv[4]));sys.exit(int(sys.argv[3]));'
        wire = (json.dumps(STATE)+'\n').encode()+tail
        node = subprocess.Popen([sys.executable, '-c', code, str(b.fileno()), wire.hex(),
                                 str(int(exit_signal) if exit_signal is not None else exit_code), str(silent_seconds)],
                                pass_fds=(b.fileno(),))
        b.close()
        def cleanup():
            if node.poll() is None:
                node.kill()
            node.wait(timeout=3)
        self.addCleanup(cleanup)
        return launcher.Channel(a), node

    def test_complete_feedback_and_ipc_failure_survive_nonzero_native_exit(self):
        channel, node = self.peer(tail=b'{"kind":', exit_code=7)
        rows = []
        state, _, _ = standby.monitor(channel, node, threading.Event(), rows.append)
        self.assertEqual(state.get('relative_height_m'), .42)
        self.assertEqual(rows[-1].get('ipc_error', {}).get('kind'), 'truncated_json')
        self.assertEqual(rows[-1].get('controller_returncode'), 7)
        self.assertEqual(rows[-1].get('controller_exit_code'), 7)
        self.assertIsNone(rows[-1].get('controller_exit_signal'))
        self.assertIs(rows[-1].get('feedback_current'), False)

    def test_clean_eof_is_recorded_separately_from_normal_native_exit(self):
        channel, node = self.peer(exit_code=0)
        rows = []
        standby.monitor(channel, node, threading.Event(), rows.append)
        self.assertEqual(rows[-1].get('ipc_error', {}).get('kind'), 'clean_eof')
        self.assertEqual(rows[-1].get('controller_returncode'), 0)
        self.assertEqual(rows[-1].get('controller_exit_code'), 0)

    def test_native_signal_exit_keeps_signal_identity(self):
        channel, node = self.peer(exit_signal=signal.SIGUSR1)
        rows = []
        standby.monitor(channel, node, threading.Event(), rows.append)
        self.assertEqual(rows[-1].get('controller_returncode'), -signal.SIGUSR1)
        self.assertEqual(rows[-1].get('controller_exit_signal'), signal.SIGUSR1)
        self.assertEqual(rows[-1].get('controller_exit_signal_name'), 'SIGUSR1')
        self.assertIsNone(rows[-1].get('controller_exit_code'))

    def test_invalid_tail_stops_live_peer_without_waiting_for_another_read_event(self):
        channel, node = self.peer(tail=b'[]\n', exit_code=9, silent_seconds=.5,
                                  keep_open=True, exit_on_stop=True)
        rows = []
        state, _, _ = standby.monitor(channel, node, threading.Event(), rows.append)
        self.assertEqual(state.get('relative_height_m'), .42)
        self.assertEqual(rows[-1].get('ipc_error', {}).get('kind'), 'invalid_message')
        self.assertEqual(node.returncode, 0, 'native must receive the graceful stop before its timeout')

    def test_silent_live_controller_cannot_refresh_aircraft_feedback(self):
        channel, node = self.peer(silent_seconds=3.25)
        rows = []
        standby.monitor(channel, node, threading.Event(), rows.append)
        live_stale = [row for row in rows if row['status'] == 'WAITING'
                      and (row.get('state_age_seconds') or 0) > 3]
        self.assertTrue(live_stale, rows)
        self.assertIs(live_stale[0].get('feedback_current'), False)
        self.assertEqual(live_stale[0]['state']['relative_height_m'], .42)

    def test_retry_retains_history_age_ipc_error_and_native_exit(self):
        with tempfile.TemporaryDirectory() as folder:
            home = Path(folder)
            a, b = socket.socketpair()
            self.addCleanup(a.close)
            self.addCleanup(b.close)
            binary = home/'native_fixture'
            wire = (json.dumps(STATE)+'\n').encode().hex()
            binary.write_text('#!/usr/bin/env python3\nimport signal,socket,sys,time\n'
                              'signal.signal(signal.SIGTERM,lambda *_:None)\n'
                              's=socket.socket(fileno=int(sys.argv[-1]))\n'
                              f's.sendall(bytes.fromhex({wire!r}))\n'
                              's.shutdown(socket.SHUT_WR)\ntime.sleep(.08)\nsys.exit(7)\n')
            binary.chmod(0o700)
            retries = []
            class StopOnRetries(threading.Event):
                def wait(self, timeout=None):
                    if timeout == 5:
                        end=time.monotonic()+1
                        while True:
                            try:row=json.loads((home/'state/status.json').read_text())
                            except (OSError,ValueError):row={}
                            newer=not retries or row.get('updated_unix',0)>retries[-1].get('updated_unix',0)
                            if newer and row.get('controller_returncode')==7 and 'retry_in_seconds' in row and (attempts[0]<=1 or 'UART disappeared' in row.get('reason','')):
                                break
                            if time.monotonic()>=end:break
                            time.sleep(.002)
                        retries.append(row)
                        if len(retries) == 8:
                            self.set()
                        else:
                            time.sleep(.03)
                        return self.is_set()
                    return super().wait(timeout)
            existing = launcher.Process(os.getpid(), os.getuid(), ('fixture-agent',))
            real_popen, real_read = subprocess.Popen, Path.read_bytes
            attempts = [0]
            def preflight(*_):
                attempts[0] += 1
                if attempts[0] > 1:
                    raise launcher.ControlError('UART disappeared during reconnect')
                return binary, existing
            def spawn(argv, **kwargs):
                return real_popen([*argv, str(b.fileno())], pass_fds=(b.fileno(),), **kwargs)
            def read_bytes(path):
                if path == Path('/proc', str(existing.pid), 'cmdline'):
                    return b'fixture-agent\0'
                return real_read(path)
            with patch.object(Path, 'home', return_value=home), \
                 patch.object(standby, 'preflight', side_effect=preflight), \
                 patch.object(standby, 'validate', return_value={}), \
                 patch.object(standby.subprocess, 'Popen', side_effect=spawn), \
                 patch.object(standby, 'connect_controller', return_value=launcher.Channel(a)), \
                 patch.object(Path, 'read_bytes', new=read_bytes):
                standby.supervise(home/'project', home/'agent', home/'state', StopOnRetries())
            self.assertEqual(len(retries), 8)
            for row in retries:
                self.assertEqual(row.get('state', {}).get('relative_height_m'), .42)
                self.assertIs(row.get('feedback_current'), False)
                self.assertEqual(row.get('ipc_error', {}).get('kind'), 'clean_eof')
                self.assertEqual(row.get('controller_returncode'), 7)
                self.assertIn('7', row['reason'])
            self.assertGreater(retries[1]['state_age_seconds'], retries[0]['state_age_seconds'])
            self.assertIn('UART disappeared during reconnect', retries[-1]['reason'])
            final = json.loads((home/'state/status.json').read_text())
            self.assertEqual(final['status'], 'STOPPED')
            self.assertEqual(final.get('state', {}).get('relative_height_m'), .42)
            self.assertIs(final.get('feedback_current'), False)

    def test_view_labels_history_even_when_supervisor_snapshot_is_fresh(self):
        row = dict(status='WAITING', reason='retry after disconnect', supervisor_pid=10,
                   updated_unix=1000, state=STATE, state_age_seconds=20, feedback_current=False)
        view = navigation.render_status(row, dict(ActiveState='active', MainPID='10'), now=1001)
        self.assertIn('历史', view)
        self.assertIn('Offboard', view)
        self.assertIn('0.42 m', view)
        self.assertIn('当前飞机状态未知', view)
        self.assertNotIn('实际模式：', view)
        self.assertIn('21.0', view)

    def test_view_adds_snapshot_elapsed_time_to_feedback_age(self):
        row = dict(status='HEIGHT_HOLD', reason='hold', supervisor_pid=10, updated_unix=1000,
                   state=STATE, state_age_seconds=2.5, feedback_current=True)
        view = navigation.render_status(row, dict(ActiveState='active', MainPID='10'), now=1001)
        self.assertIn('历史', view)
        self.assertNotIn('实际模式：', view)
        self.assertNotIn('正在定高', view)
        self.assertIn('[STALE]', view)

    def test_view_requires_native_fresh_flags_and_explains_missing_height(self):
        row = dict(status='WAITING', reason='feedback not ready', supervisor_pid=10,
                   updated_unix=1000, state=dict(STATE, local_fresh=False, relative_height_m=None),
                   state_age_seconds=.1, feedback_current=True)
        view = navigation.render_status(row, dict(ActiveState='active', MainPID='10'), now=1001)
        self.assertNotIn('实际模式：', view)
        self.assertIn('relative_height_m', view)

    def test_view_shows_native_height_reason_and_expected_observed_slots(self):
        row = dict(status='WAITING', reason='ground reference pending', supervisor_pid=10,
                   updated_unix=1000, state=dict(STATE, relative_height_m=None,
                       height_unavailable_reason='REFERENCE_NOT_READY',
                       rc_navigation_mode_slot=1, rc_mode_slot=2),
                   state_age_seconds=.1, feedback_current=True)
        view = navigation.render_status(row, dict(ActiveState='active', MainPID='10'), now=1001)
        self.assertIn('REFERENCE_NOT_READY', view)
        self.assertIn('期望 1 / 观测 2', view)

    def test_view_shows_current_feedback_only_with_valid_age_and_native_flags(self):
        row = dict(status='HEIGHT_HOLD', reason='hold', supervisor_pid=10, updated_unix=1000,
                   state=STATE, state_age_seconds=.1, feedback_current=True)
        view = navigation.render_status(row, dict(ActiveState='active', MainPID='10'), now=1001)
        self.assertIn('实际模式：Offboard', view)
        self.assertIn('反馈年龄：1.1 秒', view)
        self.assertIn('正在定高', view)
        self.assertNotIn('历史', view)


if __name__ == '__main__':
    unittest.main()
