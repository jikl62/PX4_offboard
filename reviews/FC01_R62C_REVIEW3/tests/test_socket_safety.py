"""Offline filesystem/update fixtures; no ROS, no real FC01/current/service."""
import errno
import importlib.util
import os
from pathlib import Path
import shutil
import socket
import stat
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[1]
def module(name):
    if name in sys.modules:return sys.modules[name]
    if str(ROOT/'tools') not in sys.path:sys.path.insert(0,str(ROOT/'tools'))
    spec=importlib.util.spec_from_file_location(name,ROOT/'tools'/f'{name}.py')
    obj=importlib.util.module_from_spec(spec);sys.modules[name]=obj;spec.loader.exec_module(obj);return obj

class SocketSafetyTests(unittest.TestCase):
    def fixture(self,p):
        base=p/'system';old=base/'releases/old';new=base/'releases/new'
        for release in (old,new):
            node=release/'workspace/install/jikl_fc01_system/lib/jikl_fc01_system/fc01_system_node'
            node.parent.mkdir(parents=True);node.write_text('#!/bin/bash\nexit 0\n');node.chmod(0o755)
        (base/'current').symlink_to(old)
        bindir=p/'bin';bindir.mkdir();(bindir/'systemctl').write_text('#!/bin/bash\nexit 3\n');(bindir/'systemctl').chmod(0o755)
        env=dict(os.environ,FC01_HOME=str(base),FC01_SOCKET=str(p/'control.sock'),PATH=str(bindir)+':'+os.environ['PATH'])
        return base,old,new,env
    def test_update_rejects_regular_file_symlink_fifo_never_deletes(self):
        for kind in ('file','symlink','fifo'):
            with self.subTest(kind=kind),tempfile.TemporaryDirectory() as directory:
                p=Path(directory);base,old,new,env=self.fixture(p);path=p/'control.sock'
                if kind=='file':path.write_text('retain')
                elif kind=='symlink':path.symlink_to(p/'absent')
                else:os.mkfifo(path)
                before=path.lstat()
                result=subprocess.run(['bash',str(ROOT/'scripts/update.sh'),str(new)],env=env,capture_output=True,text=True,timeout=3)
                self.assertNotEqual(result.returncode,0,result.stdout+result.stderr)
                self.assertEqual((base/'current').resolve(),old)
                self.assertEqual(path.lstat().st_ino,before.st_ino)
    def test_policy_helper_required(self):
        self.assertTrue((ROOT/'tools/socket_safety.py').is_file(),'safe socket policy/diagnostic is absent')
    def test_stale_looking_socket_diagnostic_only_retained(self):
        if not (ROOT/'tools/socket_safety.py').is_file():self.skipTest('RED: helper not implemented yet')
        safe=module('socket_safety')
        with tempfile.TemporaryDirectory() as directory:
            p=Path(directory);path=p/'control.sock'
            try:os.mknod(path,stat.S_IFSOCK|0o600)
            except PermissionError:self.skipTest('native socket filesystem fixture unavailable: UNKNOWN')
            before=path.lstat();message=safe.socket_diagnosis(path)
            self.assertIn('LIVENESS_UNKNOWN',message)
            self.assertIn('rm --',message)
            with self.assertRaises(safe.SocketSafetyError):
                with safe.SocketLease(path):safe.require_absent_socket(path)
            self.assertEqual(path.lstat().st_ino,before.st_ino)
    def test_lock_busy_changed_path_unknown_service_fail_closed(self):
        if not (ROOT/'tools/socket_safety.py').is_file():self.skipTest('RED: helper not implemented yet')
        safe=module('socket_safety');update=module('update')
        with tempfile.TemporaryDirectory() as directory:
            p=Path(directory);path=p/'control.sock'
            with safe.SocketLease(path) as lease:
                with self.assertRaises(safe.SocketSafetyError):
                    with safe.SocketLease(path):pass
                lock=Path(str(path)+'.lock');lock.unlink();lock.write_text('replaced')
                with self.assertRaises(safe.SocketSafetyError):lease.verify_identity()
            with patch.object(update.subprocess,'run',return_value=type('Result',(),{'returncode':1})()):
                with self.assertRaises(safe.SocketSafetyError):update.require_inactive_service()
    def test_socket_appearing_during_activation_refuses(self):
        if not (ROOT/'tools/socket_safety.py').is_file():self.skipTest('RED: helper not implemented yet')
        update=module('update');safe=module('socket_safety')
        with tempfile.TemporaryDirectory() as directory:
            p=Path(directory);base,old,new,_=self.fixture(p);path=p/'control.sock';calls=[]
            def service_check():
                calls.append(1)
                if len(calls)==2:path.write_text('raced')
            with patch.object(update,'require_inactive_service',side_effect=service_check):
                with self.assertRaises(safe.SocketSafetyError):update.activate(base,new,path)
            self.assertEqual((base/'current').resolve(),old);self.assertEqual(path.read_text(),'raced')
    def test_live_listener_refuses_update(self):
        with tempfile.TemporaryDirectory() as directory:
            p=Path(directory);base,old,new,env=self.fixture(p);path=p/'control.sock'
            try:listener=socket.socket(socket.AF_UNIX,socket.SOCK_STREAM)
            except PermissionError:self.skipTest('native AF_UNIX listener forbidden: UNKNOWN')
            with listener:
                try:listener.bind(str(path));listener.listen(1)
                except PermissionError:self.skipTest('native AF_UNIX listener forbidden: UNKNOWN')
                result=subprocess.run(['bash',str(ROOT/'scripts/update.sh'),str(new)],env=env,capture_output=True,text=True,timeout=3)
                self.assertNotEqual(result.returncode,0);self.assertEqual((base/'current').resolve(),old);self.assertTrue(path.exists())

if __name__=='__main__':unittest.main()
