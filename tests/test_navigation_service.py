import importlib
import io
import contextlib
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

ROOT=Path(__file__).resolve().parents[1]

class NavigationServiceTests(unittest.TestCase):
    def api(self):
        self.assertTrue((ROOT/'navigation_service.py').exists(),'navigation background controller missing')
        return importlib.import_module('navigation_service')

    def test_navigation_profile_keeps_program_arm_disabled_and_numeric_limits(self):
        m=self.api();base=yaml.safe_load((ROOT/'tests/fixtures/base.yaml').read_text())
        result=m.navigation_profile(base)['fc01_system']['ros__parameters']
        self.assertTrue(result['rc_navigation_enabled'])
        self.assertTrue(result['flight_actions_enabled'] and result['takeoff_enabled'])
        self.assertFalse(result['arm_enabled'] or result['flight_enabled'] or result['bench_arm_only_enabled'])
        self.assertTrue(result['rc_takeover_confirmed'] and result['flight_parameters_confirmed'])
        self.assertFalse(result['autonomous_failsafe_confirmed'])
        self.assertEqual(result['rc_target_height'],.5)
        self.assertEqual(result['rc_navigation_mode_slot'],6)
        self.assertEqual(result['csv_path'],'')
        for name in ('aa_hover_thrust','aa_takeoff_reserve_thrust','aa_max_tilt_deg','aa_feedback_rate_tolerance_mps','aa_feedback_dwell_seconds'):
            self.assertEqual(result[name],base['fc01_system']['ros__parameters'][name])

    def test_mode_slot_is_configurable_and_rejects_invalid_values(self):
        m=self.api();base=yaml.safe_load((ROOT/'tests/fixtures/base.yaml').read_text())
        for slot in range(1,7):
            with self.subTest(slot=slot):
                p=m.navigation_profile(base,mode_slot=slot)['fc01_system']['ros__parameters']
                self.assertEqual(p['rc_navigation_mode_slot'],slot)
                self.assertFalse(p['arm_enabled'])
        for slot in (0,7,-1,True,1.0,'1',None):
            with self.subTest(invalid=slot),self.assertRaises(m.ControlError):
                m.navigation_profile(base,mode_slot=slot)

    def test_load_profile_uses_settings_slot_and_preserves_base_when_omitted(self):
        m=self.api();base=yaml.safe_load((ROOT/'tests/fixtures/base.yaml').read_text())
        base['fc01_system']['ros__parameters']['rc_navigation_mode_slot']=2
        with tempfile.TemporaryDirectory() as d,patch.object(m.standby,'validate',return_value=base):
            entry=Path(d);path=entry/'navigation_settings.json'
            path.write_text(json.dumps({'target_height_m':.5,'navigation_mode_slot':1}))
            self.assertEqual(m.load_profile(entry,entry/'agent',entry)['fc01_system']['ros__parameters']['rc_navigation_mode_slot'],1)
            path.write_text(json.dumps({'target_height_m':.5}))
            self.assertEqual(m.load_profile(entry,entry/'agent',entry)['fc01_system']['ros__parameters']['rc_navigation_mode_slot'],2)

    def test_historical_phase_cannot_override_retry_or_channel_failure(self):
        m=self.api()
        for phase in ('SAFETY','RC_READY_WAIT_ARM','HEIGHT_HOLD'):
            row=dict(status='WAITING',reason='truncated_json; native exit 2',feedback_current=False,
                     state=dict(phase=phase,reason='previous flight reason'))
            described=m.describe_row(row)
            self.assertEqual(described['status'],'WAITING')
            self.assertEqual(described['reason'],'truncated_json; native exit 2')

    def test_native_fault_overrides_connected_and_ready_is_distinct(self):
        m=self.api()
        common=dict(status='CONNECTED',reason='transport okay',state=dict(phase='SAFETY',reason='VERTICAL_VELOCITY_INCONSISTENT'))
        row=m.describe_row(common)
        self.assertEqual(row['status'],'FAULT')
        self.assertIn('VERTICAL_VELOCITY_INCONSISTENT',row['reason'])
        common['state']=dict(phase='RC_READY_WAIT_ARM',reason='RC_READY_WAIT_CH5_MANUAL_ARM')
        self.assertEqual(m.describe_row(common)['status'],'READY_WAIT_ARM')
        common['state']=dict(phase='HEIGHT_HOLD',reason='HEIGHT_HOLD')
        self.assertEqual(m.describe_row(common)['status'],'HEIGHT_HOLD')

    def test_service_runs_private_native_with_status_only_and_graceful_recovery(self):
        m=self.api();import standby
        from launcher import Process,Channel
        with tempfile.TemporaryDirectory() as d:
            home=Path(d);entry=home/'entry';entry.mkdir()
            native=entry/'native_fixture';native.write_text((ROOT/'tests/navigation_peer.py').read_text());native.chmod(0o700)
            a,b=socket.socketpair();self.addCleanup(a.close);self.addCleanup(b.close)
            agent=subprocess.Popen([sys.executable,'-c','import time;time.sleep(30)'])
            def cleanup():
                if agent.poll() is None:agent.terminate()
                agent.wait(timeout=3)
            self.addCleanup(cleanup)
            existing=Process(agent.pid,os.getuid(),tuple(agent.args))
            base=yaml.safe_load((ROOT/'tests/fixtures/base.yaml').read_text())
            stopped=threading.Event();timer=threading.Timer(.45,stopped.set);timer.start();self.addCleanup(timer.cancel)
            real_popen=subprocess.Popen;real_read=Path.read_bytes
            def spawn(argv,**kwargs):return real_popen([*argv,'--test-fd='+str(b.fileno())],**kwargs,pass_fds=(b.fileno(),))
            def read_bytes(path):
                if path==Path('/proc',str(agent.pid),'cmdline'):return b'\0'.join(os.fsencode(x) for x in agent.args)+b'\0'
                return real_read(path)
            start=time.monotonic()
            with patch.object(Path,'home',return_value=home),patch.object(standby,'preflight',return_value=(home/'old_binary',existing)),\
                 patch.object(standby,'verify_binary'),patch.object(standby.subprocess,'Popen',side_effect=spawn),\
                 patch.object(standby,'connect_controller',return_value=Channel(a)),patch.object(Path,'read_bytes',new=read_bytes):
                standby.supervise(home/'base',home/'agent',home/'FC01_navigation',stopped,
                    controller=native,config_factory=lambda *_:m.navigation_profile(base),
                    status_metadata={'rc_navigation_enabled':True,'flight_actions_enabled':True},row_formatter=m.describe_row)
            self.assertGreater(time.monotonic()-start,.7,'native recovery must outlive supervisor stop request')
            self.assertIsNone(agent.poll())
            trace=json.loads((home/'FC01_navigation/trace.json').read_text())
            self.assertEqual(trace['ops'],['STATUS'])
            self.assertTrue(trace['rc_enabled'] and trace['automatic_arm_disabled'])
            self.assertTrue(trace['external_log_disabled'])
            last=json.loads((home/'FC01_navigation/status.json').read_text())
            self.assertEqual(last['status'],'STOPPED')
            self.assertTrue(last['rc_navigation_enabled'] and last['flight_actions_enabled'])

    def test_current_service_instance_controls_snapshot_validity(self):
        self.api()
        self.assertTrue((ROOT/'navigation.py').exists(),'navigation management/view missing')
        m=importlib.import_module('navigation')
        row=dict(status='READY_WAIT_ARM',reason='ready',supervisor_pid=10,updated_unix=1000,
                 state=dict(phase='RC_READY_WAIT_ARM',armed=False,nav_state=15,relative_height_m=.01))
        self.assertIn('STARTING',m.render_status(row,{'ActiveState':'active','MainPID':'11'},now=1001))
        self.assertIn('STALE',m.render_status(row,{'ActiveState':'active','MainPID':'10'},now=1020))
        self.assertIn('READY_WAIT_ARM',m.render_status(row,{'ActiveState':'active','MainPID':'10'},now=1001))
        self.assertIn('STOPPED',m.render_status(row,{'ActiveState':'inactive','MainPID':'0'},now=1001))

    def test_role_metadata_does_not_override_actual_native_gate_values(self):
        self.api();import standby
        with tempfile.TemporaryDirectory() as d:
            path=Path(d)
            standby.write_status(path,{'status':'WAITING','rc_navigation_enabled':True,'flight_actions_enabled':True})
            row=json.loads((path/'status.json').read_text())
            self.assertTrue(row['rc_navigation_enabled'] and row['flight_actions_enabled'])

    def test_view_rejects_previous_boot_and_malformed_time(self):
        self.api();m=importlib.import_module('navigation')
        row=dict(status='READY_WAIT_ARM',reason='ready',supervisor_pid=10,updated_unix=1000,boot_id='old')
        info={'ActiveState':'active','MainPID':'10','boot_id':'new'}
        self.assertIn('STARTING',m.render_status(row,info,now=1001))
        row['boot_id']='new';row['updated_unix']='bad timestamp'
        self.assertIn('STALE',m.render_status(row,info,now=1001))

    def test_closed_read_only_view_does_not_stop_or_manage_service(self):
        self.api();m=importlib.import_module('navigation')
        class ClosedOutput:
            def isatty(self):return False
            def write(self,_):raise BrokenPipeError('view closed')
            def flush(self):pass
        row=dict(supervisor_pid=10,status='WAITING',reason='ready',updated_unix=time.time())
        with patch.object(m,'service_info',return_value={'ActiveState':'active','MainPID':'10'}),\
             patch.object(m,'snapshot',return_value=row),\
             patch.object(m.subprocess,'run',side_effect=AssertionError('view cannot issue service operations')),\
             contextlib.redirect_stdout(ClosedOutput()):
            m.monitor(Path('/unused'))

import test_native_profile as native_contract
class NavigationNativeConfigContract(native_contract.NativeProfileContract):
    def test_actual_generated_rc_configuration_is_valid(self):
        m=importlib.import_module('navigation_service')
        p=m.navigation_profile(yaml.safe_load((ROOT/'tests/fixtures/base.yaml').read_text()))['fc01_system']['ros__parameters']
        result=self.run_native(p)
        self.assertEqual(result.returncode,0,result.stderr)
    def test_actual_native_rejects_automatic_arm_in_manual_route(self):
        m=importlib.import_module('navigation_service')
        p=m.navigation_profile(yaml.safe_load((ROOT/'tests/fixtures/base.yaml').read_text()))['fc01_system']['ros__parameters']
        p['arm_enabled']=True
        self.assertNotEqual(self.run_native(p).returncode,0)
        p['arm_enabled']=False
        for slot in (0,7):
            p['rc_navigation_mode_slot']=slot
            self.assertNotEqual(self.run_native(p).returncode,0)

    def test_all_valid_mode_slots_are_accepted_by_actual_native_config(self):
        m=importlib.import_module('navigation_service')
        p=m.navigation_profile(yaml.safe_load((ROOT/'tests/fixtures/base.yaml').read_text()))['fc01_system']['ros__parameters']
        for slot in range(1,7):
            with self.subTest(slot=slot):
                p['rc_navigation_mode_slot']=slot
                result=self.run_native(p)
                self.assertEqual(result.returncode,0,result.stderr)

if __name__=='__main__':unittest.main()
