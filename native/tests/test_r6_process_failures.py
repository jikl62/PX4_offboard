"""Local OS signals only; external PX4 recovery deliberately UNVERIFIED."""
import json
import resource
import signal
import subprocess
import tempfile
from pathlib import Path
import unittest
ROOT=Path(__file__).resolve().parents[1]
class ProcessFailureTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp=tempfile.TemporaryDirectory(prefix='r6_signal_fixture_')
        cls.exe=Path(cls.tmp.name)/'fault_child'
        subprocess.run(['g++','-std=c++17','-Wall','-Wextra','-Werror','-pedantic',
                        '-I',str(ROOT/'src/jikl_fc01_system/include'),'-I',str(ROOT/'tests'),
                        str(ROOT/'tests/r6_fault_child.cpp'),'-o',str(cls.exe)],check=True)
    @classmethod
    def tearDownClass(cls):cls.tmp.cleanup()
    def child(self,scenario=None):
        def no_core():resource.setrlimit(resource.RLIMIT_CORE,(0,0))
        p=subprocess.Popen([str(self.exe)]+([scenario] if scenario else []),stdout=subprocess.PIPE,
                           stderr=subprocess.PIPE,text=True,preexec_fn=no_core)
        self.assertEqual(p.stdout.readline().strip(),'READY')
        return p
    def graceful(self,sig):
        p=self.child();p.send_signal(sig);out,err=p.communicate(timeout=8)
        self.assertEqual(p.returncode,0,err)
        rows=[json.loads(line) for line in out.splitlines()]
        active=[r for r in rows if r['shutting_down'] and r['heartbeat']]
        self.assertGreater(len(active),3)
        self.assertTrue(all(r['target'] for r in active))
        self.assertEqual(rows[-1]['phase'],'RECOVERY_DESCENT')
        self.assertTrue(rows[-1]['recovery_budget_exhausted'])
        self.assertTrue(all(r['target'] and r['heartbeat'] for r in rows))
        self.assertFalse(rows[-1]['px4_fallback_verified'])
    def test_sigint_retains_healthy_custody_after_budget(self):self.graceful(signal.SIGINT)
    def test_sigterm_retains_healthy_custody_after_budget(self):self.graceful(signal.SIGTERM)
    def test_sigkill_cannot_claim_companion_recovery(self):
        p=self.child();p.kill();out,err=p.communicate(timeout=8)
        self.assertEqual(p.returncode,-signal.SIGKILL,err)
        self.assertFalse(any(json.loads(x)['px4_fallback_verified'] for x in out.splitlines()))
    def test_crash_cannot_claim_companion_recovery(self):
        p=self.child('crash');out,err=p.communicate(timeout=8)
        self.assertEqual(p.returncode,-signal.SIGABRT,err)
        self.assertEqual(out,'')
if __name__=='__main__':unittest.main()
