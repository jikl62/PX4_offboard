import contextlib
import importlib.util
import io
import json
import os
from pathlib import Path
import threading
import time
import tty
from types import SimpleNamespace
import unittest
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[1]
GOOD={
    'kind':'state','phase':'OFFBOARD_READY','nav_state':14,'armed':False,'landed':True,'at_rest':True,
    'status_fresh':True,'local_fresh':True,'land_fresh':True,'failsafe_fresh':True,'battery_fresh':True,
    'attitude_fresh':True,'z_valid':True,'vz_valid':True,'preflight':True,'battery_connected':True,
    'profile':'attitude_altitude','bench_arm_only_enabled':True,
    'source_clock_valid':True,'source_clock_validated':True,
    'authority_mode':'RC_TAKEOVER','authority_attempt_mode':'RC_TAKEOVER','authority_locked':True,
    'authority_ready':True,'rc_link_ready':True,'rc_switches_ready':True,'rc_kill_latched':False,
    'flight_actions_enabled':True,'arm_enabled':True,'takeoff_enabled':False,
    'flight_parameters_confirmed':True,'publication_fault_latched':False,
    'stationary_seconds':3.,'required_ground_seconds':2.,'ground_vz_ned':0.,'ground_vz_tolerance':.1,
    'authority_ground_failed_mask':0,'estimate_valid':False,'ground_test':'NOT_RUN','reason':'OFFBOARD_READY',
    'authority_reason':'AUTHORITY_READY','takeover_reason':'',
}

def console_module():
    spec=importlib.util.spec_from_file_location('review_console',ROOT/'tools/fcctl.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module);return module

class ReviewConsoleTests(unittest.TestCase):
    def exercise(self,state):
        module=console_module();ops=[];ready=threading.Event();input_master,input_slave=os.openpty()
        tty.setraw(input_slave)
        stdin=os.fdopen(input_slave,'r',buffering=1)
        class PipeTransport:
            def __init__(self,*args):self.read,self.write=os.openpty();tty.setraw(self.write)
            def fileno(self):return self.read
            def settimeout(self,*args):pass
            def connect(self,*args):pass
            def setblocking(self,*args):pass
            def send(self,data):
                for line in data.decode().splitlines():
                    seq,op,*_=line.split();ops.append(op)
                    response={'kind':'reply','seq':int(seq),'accepted':True,'message':'STATUS' if op=='STATUS' else 'ALIVE'}
                    wire=json.dumps(response)+'\n'+json.dumps(state)+'\n'
                    os.write(self.write,wire.encode());ready.set()
                return len(data)
            def recv(self,n):return os.read(self.read,n)
            def __enter__(self):return self
            def __exit__(self,*args):os.close(self.read);os.close(self.write)
        errors=[]
        def operator():
            try:
                if not ready.wait(2):raise AssertionError('console did not connect')
                for command in ('arm-only','BENCH ARM CONFIRM','quit'):
                    time.sleep(.12);os.write(input_master,(command+'\n').encode())
            except Exception as error:errors.append(error)
        thread=threading.Thread(target=operator);thread.start();output=io.StringIO()
        args=SimpleNamespace(command=None,socket='pipe-fixture',timeout=2,json=False,bench_arm_only=True)
        try:
            with patch.object(module.socket,'socket',PipeTransport),patch.object(module.sys,'stdin',stdin),contextlib.redirect_stdout(output):
                result=module.run(args)
            thread.join(2)
            self.assertFalse(errors);self.assertEqual(result,0);self.assertIn('PING',ops)
            return ops,output.getvalue()
        finally:
            thread.join(2);stdin.close();os.close(input_master)

    def test_valid_interaction_sends_one_arm_after_confirmation(self):
        ops,_=self.exercise(dict(GOOD));self.assertEqual(ops.count('ARM_ONLY'),1)

    def test_unverified_bench_or_stale_source_never_reaches_arm(self):
        for field,value in (('bench_arm_only_enabled',False),('profile','ground'),
                            ('land_fresh',False),('source_clock_validated',False),('armed',0)):
            with self.subTest(field=field):
                ops,_=self.exercise(dict(GOOD,**{field:value}));self.assertNotIn('ARM_ONLY',ops)

    def test_report_rejected_ground_sample_with_actual_limits(self):
        module=console_module();output=io.StringIO()
        reply=dict(GOOD,kind='reply',accepted=False,message='AUTHORITY_SAFE_GROUND_REQUIRED',
                   last_authority_ground_evidence_valid=True,last_authority_ground_failed_mask=24,
                   last_authority_ground_stationary_seconds=.75,last_authority_ground_vz_ned=.12)
        with contextlib.redirect_stdout(output):module.show(reply,False)
        self.assertIn('0.75',output.getvalue());self.assertIn('0.12',output.getvalue())

if __name__=='__main__':unittest.main()
