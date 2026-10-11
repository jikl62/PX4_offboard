import importlib.util
import json
import contextlib
import errno
import io
import os
import shutil
import shlex
from pathlib import Path
import socket
import subprocess
import sys
import tempfile
import threading
import time
import tty
import unittest
from unittest.mock import patch
from types import SimpleNamespace
import zipfile

ROOT=Path(__file__).resolve().parents[1]
def module(name):
    spec=importlib.util.spec_from_file_location(name,ROOT/'tools'/f'{name}.py')
    result=importlib.util.module_from_spec(spec);spec.loader.exec_module(result);return result

class ToolsTests(unittest.TestCase):
    def test_install_never_switches_current_and_explicit_rollback(self):
        # External platform/ROS tools are fixtures; this is not a ROS build test.
        for existing in (False,True):
            with tempfile.TemporaryDirectory() as directory:
                p=Path(directory);source=p/'source';base=p/'system';bin_dir=p/'bin';bin_dir.mkdir()
                shutil.copytree(ROOT,source,ignore=shutil.ignore_patterns('.git','__pycache__','core'))
                (source/'scripts/test.sh').write_text('#!/bin/bash\nexit 0\n')
                ros=p/'setup.bash';ros.write_text(':\n')
                messages=p/'msgs';messages.mkdir();(messages/'package.xml').write_text('<package/>')
                module('deploy').write_vendor_manifest(messages)
                wrapper='#!/bin/bash\nif [[ "$2" == check-platform ]]; then exit 0; fi\nexec '+shlex.quote(sys.executable)+' "$@"\n'
                (bin_dir/'python3').write_text(wrapper)
                (bin_dir/'colcon').write_text('#!/bin/bash\nset -e\nmkdir -p install/jikl_fc01_system/lib/jikl_fc01_system\nprintf "#!/bin/bash\\nexit 0\\n" > install/jikl_fc01_system/lib/jikl_fc01_system/fc01_system_node\nchmod +x install/jikl_fc01_system/lib/jikl_fc01_system/fc01_system_node\n')
                (bin_dir/'systemctl').write_text('#!/bin/bash\nexit 3\n')
                for tool in bin_dir.iterdir():tool.chmod(0o755)
                old=base/'releases/old_verified';old_node=old/'workspace/install/jikl_fc01_system/lib/jikl_fc01_system/fc01_system_node'
                old_node.parent.mkdir(parents=True);old_node.write_text('#!/bin/bash\nexit 0\n');old_node.chmod(0o755)
                if existing:(base/'current').symlink_to(old)
                env=dict(os.environ,PATH=str(bin_dir)+':'+os.environ['PATH'],FC01_CLEAN_BUILD='1',FC01_HOME=str(base),ROS_SETUP=str(ros),FC01_SOCKET=str(p/'absent.sock'))
                install=subprocess.run(['bash',str(source/'scripts/install.sh'),str(messages)],env=env,capture_output=True,text=True)
                self.assertEqual(install.returncode,0,install.stdout+install.stderr)
                if existing:self.assertEqual((base/'current').resolve(),old)
                else:self.assertFalse((base/'current').exists() or (base/'current').is_symlink())
                release=Path((base/'built-release').read_text().strip())
                self.assertTrue(release.name.startswith('0.4.4-offboard-flight-dev-'))
                self.assertTrue((release/'config/attitude_altitude.yaml').is_file())
                for target in (release,old):
                    update=subprocess.run(['bash',str(source/'scripts/update.sh'),str(target)],env=env,capture_output=True,text=True)
                    self.assertEqual(update.returncode,0,update.stdout+update.stderr)
                    self.assertEqual((base/'current').resolve(),target)

    def test_ready_command(self):
        self.assertEqual(module('fcctl').parse_command('attitude-ready'),('ATTITUDE_READY',[],0.))

    def test_single_flight_commands_never_connect(self):
        for op in ('takeoff 1','vel 0 0 0 0 1','hold','land'):
            result=subprocess.run([sys.executable,str(ROOT/'tools/fcctl.py'),'--socket','/no/such/controller','--command',op],capture_output=True,text=True)
            self.assertEqual(result.returncode,2)
            self.assertIn('live interactive console',result.stderr)

    def test_ready_client_waits_for_confirmed_state(self):
        with tempfile.TemporaryDirectory() as directory:
            path=str(Path(directory)/'control.sock');received=[];errors=[]
            def server():
                try:
                    with socket.socket(socket.AF_UNIX,socket.SOCK_STREAM) as listener:
                        listener.bind(path);listener.listen(1)
                        with listener.accept()[0] as client:
                            received.append(client.recv(4096).decode())
                            client.sendall(b'{"kind":"reply","seq":1,"accepted":true,"message":"pending"}\n')
                            time.sleep(.12)
                            client.sendall(b'{"kind":"state","phase":"OFFBOARD_READY","armed":false,"nav_state":14,"estimate_valid":false,"ground_test":"NOT_RUN","reason":"confirmed"}\n')
                except Exception as error:errors.append(str(error))
            worker=threading.Thread(target=server);worker.start()
            for _ in range(100):
                if Path(path).exists():break
                if errors:
                    worker.join(timeout=1)
                    self.skipTest('AF_UNIX denied by execution sandbox; native console transport UNKNOWN')
                time.sleep(.005)
            start=time.monotonic()
            result=subprocess.run([sys.executable,str(ROOT/'tools/fcctl.py'),'--socket',path,'--command','attitude-ready','--json'],capture_output=True,text=True,timeout=3)
            worker.join(timeout=1)
            self.assertEqual(errors,[])
            self.assertEqual(result.returncode,0,result.stderr)
            self.assertGreaterEqual(time.monotonic()-start,.1)
            self.assertIn('OFFBOARD_READY',result.stdout)
            self.assertEqual(received,['1 ATTITUDE_READY\n'])

    def test_ready_protocol_over_pipe_fixture(self):
        # AF_UNIX is an external OS dependency. Exercise the real select/parse/wait
        # console with pipes when sandbox policy denies socket creation.
        ctl=module('fcctl');responses=[]
        class PipeTransport:
            def __init__(self,*args):
                self.read,self.write=os.openpty();tty.setraw(self.write);self.worker=None
            def settimeout(self,*args):pass
            def connect(self,*args):pass
            def setblocking(self,*args):pass
            def fileno(self):return self.read
            def send(self,data):
                responses.append(data.decode())
                def delayed():
                    os.write(self.write,b'{"kind":"reply","seq":1,"accepted":true,"message":"pending"}\n')
                    time.sleep(.1)
                    os.write(self.write,b'{"kind":"state","phase":"OFFBOARD_READY","armed":false}\n')
                self.worker=threading.Thread(target=delayed);self.worker.start();return len(data)
            def recv(self,count):return os.read(self.read,count)
            def __enter__(self):return self
            def __exit__(self,*args):
                if self.worker:self.worker.join(timeout=1)
                os.close(self.read);os.close(self.write)
        args=SimpleNamespace(command='attitude-ready',socket='fixture',timeout=2,json=True)
        output=io.StringIO();start=time.monotonic()
        with patch.object(ctl.socket,'socket',PipeTransport),contextlib.redirect_stdout(output):
            self.assertEqual(ctl.run(args),0)
        self.assertGreaterEqual(time.monotonic()-start,.1)
        self.assertIn('OFFBOARD_READY',output.getvalue());self.assertEqual(responses,['1 ATTITUDE_READY\n'])

    def test_piped_input_is_not_a_live_console(self):
        result=subprocess.run([sys.executable,str(ROOT/'tools/fcctl.py'),'--socket','/no/such/controller'],input='takeoff 1\n',capture_output=True,text=True)
        self.assertEqual(result.returncode,2)
        self.assertIn('TTY',result.stderr)

    def test_portable_export_name_and_clean_source(self):
        deploy=module('deploy')
        with tempfile.TemporaryDirectory() as directory:
            p=Path(directory);vendor=p/'msgs';vendor.mkdir();(vendor/'package.xml').write_text('<package/>')
            deploy.write_vendor_manifest(vendor)
            output=deploy.export_bundle(vendor,p/'source.zip')
            with zipfile.ZipFile(output) as archive:
                names=archive.namelist()
                prefix='fc01_system_v'+(ROOT/'VERSION').read_text().strip()+'/'
                self.assertTrue(all(name.startswith(prefix) for name in names))
                self.assertFalse(any('/.git/' in name or '/__pycache__/' in name for name in names))
                self.assertIn(prefix+'config/attitude_altitude.yaml',names)
            deploy.validate_platform({'ID':'ubuntu','VERSION_ID':'22.04'},'aarch64')

    def test_pinned_messages_refuse_wrong_vendor_hash(self):
        deploy=module('deploy')
        with tempfile.TemporaryDirectory() as directory:
            p=Path(directory);(p/'file').write_text('first');deploy.write_vendor_manifest(p)
            (p/'file').write_text('changed')
            with self.assertRaises(ValueError):deploy.verify_vendor(p)

if __name__=='__main__':unittest.main()
