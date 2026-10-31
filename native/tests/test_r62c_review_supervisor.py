import importlib.util
import signal
import subprocess
import tempfile
from pathlib import Path
from unittest.mock import patch
import unittest

class SupervisorRecoveryTests(unittest.TestCase):
    def test_service_cannot_impose_a_second_elapsed_kill_deadline(self):
        source=Path(__file__).resolve().parents[1]/'scripts/service.sh'
        self.assertIn('TimeoutStopSec=infinity',source.read_text())

    def test_elapsed_timeout_cannot_kill_node_or_remove_its_agent(self):
        source=Path(__file__).resolve().parents[1]/'tools/start.py'
        spec=importlib.util.spec_from_file_location('review_start',source)
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        events=[];handlers={};clock=[0.]
        class Child:
            def __init__(self,name,pid):self.name=name;self.pid=pid;self.returncode=None;self.waits=0
            def poll(self):return self.returncode
            def wait(self,timeout=None):
                self.waits+=1;events.append(('wait',self.name,timeout));clock[0]+=timeout or 0
                if self.name=='node' and self.returncode is None and self.waits<=15:
                    raise subprocess.TimeoutExpired('retained recovery',timeout)
                self.returncode=0 if self.returncode is None else self.returncode
                events.append(('exit',self.name));return self.returncode
        agent=Child('agent',1001);node=Child('node',1002);children=iter([agent,node])
        def kill(pid,sig):
            events.append(('signal',pid,sig))
            if pid==node.pid and sig==signal.SIGKILL:node.returncode=-9
        def sleep(seconds):
            if seconds==.1:handlers[signal.SIGTERM](signal.SIGTERM,None)
        with tempfile.TemporaryDirectory() as folder:
            with patch.object(module.subprocess,'Popen',side_effect=lambda *a,**k:next(children)), \
                 patch.object(module.signal,'signal',side_effect=lambda s,h:handlers.__setitem__(s,h)), \
                 patch.object(module.time,'sleep',side_effect=sleep), \
                 patch.object(module.time,'monotonic',side_effect=lambda:clock[0]), \
                 patch.object(module.os,'killpg',side_effect=kill), \
                 patch.object(module.sys,'argv',['start.py']), \
                 patch.dict(module.os.environ,{'FC01_HOME':folder}):
                code=module.main()
        self.assertNotIn(('signal',node.pid,signal.SIGKILL),events)
        self.assertGreater(node.waits,15)
        self.assertLess(events.index(('exit','node')),events.index(('signal',agent.pid,signal.SIGTERM)))
        self.assertEqual(code,0)

if __name__=='__main__':unittest.main()
