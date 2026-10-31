import importlib.util
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import unittest
import json
import subprocess
import tempfile

ROOT=Path(__file__).resolve().parents[1]
def console():
    spec=importlib.util.spec_from_file_location('i_fcctl',ROOT/'tools/fcctl.py')
    obj=importlib.util.module_from_spec(spec);spec.loader.exec_module(obj);return obj

class AuthorityToolsTests(unittest.TestCase):
    def test_actual_runtime_authority_json(self):
        with tempfile.TemporaryDirectory() as directory:
            exe=Path(directory)/'status'
            subprocess.run(['g++','-std=c++17','-Wall','-Wextra','-Werror','-pedantic','-I',
                            str(ROOT/'src/jikl_fc01_system/include'),str(ROOT/'tests/authority_status_dump.cpp'),'-o',str(exe)],check=True)
            items=[json.loads(line) for line in subprocess.check_output([str(exe)],text=True).splitlines()]
        initial,selected,ready,killed,faulted=items
        self.assertEqual(initial['authority_mode'],'UNSET')
        self.assertEqual(initial['authority_reason'],'AUTHORITY_MODE_UNSET')
        self.assertEqual(selected['authority_mode'],'AUTONOMOUS_NO_RC')
        self.assertEqual(selected['authority_reason'],'AUTONOMOUS_FAILSAFE_UNCONFIRMED')
        for item in [initial,selected]:
            for field in ['authority_locked','authority_ready','rc_link_ready','rc_takeover_ready',
                          'rc_takeover_confirmed','autonomous_failsafe_ready','autonomous_failsafe_confirmed','operator_takeover_latched',
                          'rc_source_seen','rc_source_valid','rc_source_selected','rc_sticks_moving','rc_switches_seen','rc_switches_fresh','rc_switches_ready','rc_kill_latched']:
                self.assertIs(item[field],False)
            for field in ['rc_data_source','rc_mode_slot','rc_arm_switch','rc_kill_switch']:
                self.assertEqual(item[field],0)
        self.assertIs(ready['authority_ready'],True)
        self.assertEqual(ready['authority_reason'],'AUTHORITY_READY')
        self.assertIs(killed['authority_ready'],False)
        self.assertEqual(killed['authority_reason'],'OPERATOR_TAKEOVER_LATCHED')
        self.assertIs(killed['operator_takeover_latched'],False)  # pre-step observed fact
        self.assertIs(faulted['authority_ready'],False)
        self.assertIs(faulted['operator_takeover_latched'],True)
        self.assertEqual(faulted['authority_reason'],'AUTHORITY_SAFETY_RESTART_REQUIRED')
    # Removing the pre-connect one-shot guard would hit a socket side effect.
    def test_authority_console_syntax(self):
        ctl=console()
        for text,op in [('authority status','AUTHORITY_STATUS'),('authority rc','AUTHORITY_RC'),
                        ('authority autonomous','AUTHORITY_AUTONOMOUS'),('authority reset','AUTHORITY_RESET'),
                        ('authority takeover','AUTHORITY_TAKEOVER')]:
            self.assertEqual(ctl.parse_command(text),(op,[],0.))
        for text in ['authority','authority 1','authority rc extra','authority confirm']:
            with self.assertRaises(ValueError):ctl.parse_command(text)

    def test_authority_changes_one_shot_never_connect(self):
        ctl=console()
        for text in ['authority rc','authority autonomous','authority reset','authority takeover']:
            args=SimpleNamespace(command=text,socket='/must/not/connect',json=True,timeout=1.)
            with patch.object(ctl.socket,'socket',side_effect=AssertionError('socket reached')) as sock:
                with self.assertRaisesRegex(ValueError,'live interactive console'):ctl.run(args)
                sock.assert_not_called()

if __name__=='__main__':unittest.main()
