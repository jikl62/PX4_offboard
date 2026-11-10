"""Read-only report capture keeps session identity and actual file bytes."""
import hashlib
import json
import os
from pathlib import Path
import tempfile
import types
import unittest
from unittest.mock import patch
import zipfile

import navigation_report as report


class NavigationReportTests(unittest.TestCase):
    def setup_tree(self, directory):
        home=Path(directory);folder=home/'FC01_navigation';folder.mkdir()
        entry=home/'new';entry.mkdir()
        running=home/'old';running.mkdir()
        row=dict(boot_id='test-boot',controller_pid=20,entry_version='old-version',
                 config_source_path=str(running/'native/config/flight_test.yaml'),
                 state=dict(phase='SAFETY',reason='OPERATOR_TAKEOVER_LATCHED'))
        (folder/'status.json').write_text(json.dumps(row))
        (folder/'controller.log').write_text('native trace\n')
        (folder/'probe.json').write_text('{"boot_id":"another-boot"}')
        for root,marker in ((entry,'new'),(running,'old')):
            (root/'FC01_RELEASE.json').write_text(json.dumps(dict(controller_build=marker)))
            (root/'ARM64_BUILD_EVIDENCE.json').write_text(json.dumps(dict(real_build=marker)))
            (root/'ACTIVATION_STATE.json').write_text(json.dumps(dict(activation=marker)))
            logs=folder/'deploy-logs'/root.name;logs.mkdir(parents=True)
            for number in range(8):
                path=logs/(str(number)+'.log');path.write_text(str(number));os.utime(path,ns=(number,number))
        return home,folder,entry,running,row

    @staticmethod
    def reader(argv, **kwargs):
        return types.SimpleNamespace(stdout='read-only output',stderr='',returncode=0)

    def test_capture_identifies_running_and_invoked_releases_without_mutation(self):
        with tempfile.TemporaryDirectory() as directory:
            home,folder,entry,running,row=self.setup_tree(directory)
            before={str(p):p.read_bytes() for p in home.rglob('*') if p.is_file()}
            calls=[]
            def runner(argv,**kwargs):calls.append(argv);return self.reader(argv,**kwargs)
            target=report.collect_report(home,entry=entry,runner=runner)
            self.assertEqual(target.stat().st_mode&0o777,0o600)
            with zipfile.ZipFile(target) as archive:
                files={name:archive.read(name) for name in archive.namelist()}
                self.assertEqual(len(files),len(archive.namelist()))
            capture=json.loads(files['REPORT_CAPTURE.json'])
            self.assertEqual(capture['session']['controller_pid'],20)
            self.assertEqual(capture['entries'],dict(invoked_entry=str(entry),running_entry=str(running)))
            self.assertEqual(json.loads(files['running_entry/FC01_RELEASE.json'])['controller_build'],'old')
            self.assertEqual(json.loads(files['invoked_entry/ARM64_BUILD_EVIDENCE.json'])['real_build'],'new')
            self.assertTrue(capture['non_atomic_live_capture'])
            self.assertFalse(capture['status_changed_during_capture'])
            self.assertEqual(len([name for name in files if name.startswith('deploy-logs/')]),12)
            for name,item in capture['files'].items():
                if item['result']=='CAPTURED':
                    self.assertEqual(hashlib.sha256(files[name]).hexdigest(),item['sha256'])
            self.assertEqual(len(calls),2)
            self.assertIn('status',calls[0]);self.assertIn('journalctl',Path(calls[1][0]).name)
            for name,data in before.items():self.assertEqual(Path(name).read_bytes(),data)

    def test_oversized_logs_are_bounded_and_tail_is_marked(self):
        with tempfile.TemporaryDirectory() as directory:
            home,folder,entry,_,_=self.setup_tree(directory)
            (folder/'controller.log').write_bytes(b'A'*500+b'TAIL')
            with patch.object(report,'MAX_LOG',100):
                target=report.collect_report(home,entry=entry,runner=self.reader)
            with zipfile.ZipFile(target) as archive:
                self.assertEqual(archive.read('controller.log'),b'A'*96+b'TAIL')
                item=json.loads(archive.read('REPORT_CAPTURE.json'))['files']['controller.log']
                self.assertEqual(item['tail_offset'],404);self.assertTrue(item['truncated'])

    def test_missing_metadata_and_symlink_logs_do_not_stop_capture(self):
        with tempfile.TemporaryDirectory() as directory:
            home,folder,entry,_,_=self.setup_tree(directory)
            outside=home/'not-a-controller-log';outside.write_text('do not collect')
            (folder/'standby.log').symlink_to(outside)
            def failing(*args,**kwargs):raise FileNotFoundError('no systemd')
            target=report.collect_report(home,entry=entry,runner=failing)
            with zipfile.ZipFile(target) as archive:
                self.assertNotIn('standby.log',archive.namelist())
                capture=json.loads(archive.read('REPORT_CAPTURE.json'))
                self.assertEqual(capture['files']['standby.log']['result'],'UNAVAILABLE')
                self.assertIn('error',capture['commands']['service.txt'])
                self.assertIn('no systemd',archive.read('service.txt').decode())

    def test_reports_capture_live_change_instead_of_claiming_atomic_snapshot(self):
        with tempfile.TemporaryDirectory() as directory:
            home,folder,entry,_,row=self.setup_tree(directory)
            original=os.open;changed=False
            def changed_open(path,*args,**kwargs):
                nonlocal changed
                if Path(path)==folder/'runtime.yaml' and not changed:
                    changed=True;row['state']['reason']='NEW_FAULT';(folder/'status.json').write_text(json.dumps(row))
                return original(path,*args,**kwargs)
            with patch.object(report.os,'open',side_effect=changed_open):
                target=report.collect_report(home,entry=entry,runner=self.reader)
            with zipfile.ZipFile(target) as archive:
                capture=json.loads(archive.read('REPORT_CAPTURE.json'))
                self.assertTrue(capture['status_changed_during_capture'])
                self.assertNotEqual(archive.read('status.json'),archive.read('status_after_capture.json'))

    def test_full_log_budget_preserves_release_evidence_and_final_status(self):
        with tempfile.TemporaryDirectory() as directory:
            home,folder,entry,_,_=self.setup_tree(directory)
            for name in report.BASE_FILES:
                if '.log' in name:(folder/name).write_bytes(b'X'*20000)
            with patch.object(report,'MAX_TOTAL',8192),patch.object(report,'MAX_LOG',8192),patch.object(report,'MAX_METADATA',1024):
                target=report.collect_report(home,entry=entry,runner=self.reader)
            with zipfile.ZipFile(target) as archive:
                self.assertIn('invoked_entry/ARM64_BUILD_EVIDENCE.json',archive.namelist())
                self.assertIn('running_entry/FC01_RELEASE.json',archive.namelist())
                self.assertEqual(archive.read('status.json'),archive.read('status_after_capture.json'))
                capture=json.loads(archive.read('REPORT_CAPTURE.json'))
                self.assertFalse(capture['status_changed_during_capture'])
                self.assertLessEqual(capture['captured_file_bytes'],8192)

    def test_unavailable_final_status_is_unknown_instead_of_reported_changed(self):
        with tempfile.TemporaryDirectory() as directory:
            home,_,entry,_,_=self.setup_tree(directory)
            with patch.object(report,'MAX_TOTAL',1):
                target=report.collect_report(home,entry=entry,runner=self.reader)
            with zipfile.ZipFile(target) as archive:
                capture=json.loads(archive.read('REPORT_CAPTURE.json'))
                self.assertIsNone(capture['status_changed_during_capture'])


if __name__=='__main__':unittest.main()
