"""Source installer integrity/idempotence; deploy actions use a harmless stub."""
import base64
import hashlib
import importlib
import io
import json
import os
from pathlib import Path
import stat
import subprocess
import tempfile
import unittest
import zipfile

bundle=importlib.import_module('build_navigation_bundle')
ROOT=Path(__file__).resolve().parents[1]
MARKER=b'\n__FC01_PAYLOAD_BASE64__\n'


class VerticalDiagnosticBundleTests(unittest.TestCase):
    def fixture(self,folder):
        source=folder/'source';source.mkdir()
        native=source/'native/src/jikl_fc01_system'
        (native/'src').mkdir(parents=True)
        (native/'include/jikl_fc01_system').mkdir(parents=True)
        (native/'src/fc01_system_node.cpp').write_text('current controller source\n')
        (native/'src/fc01_config_check.cpp').write_text('preserved local config-check edit\n')
        for name in ('receive_events.hpp','vertical_feedback_guard.hpp','attitude_altitude_core.hpp','rc_navigation.hpp'):
            (native/'include/jikl_fc01_system'/name).write_text('current '+name+'\n')
        (source/'navigation.sh').write_text('#!/bin/bash\nexit 0\n')
        (source/'navigation_deploy.sh').write_text(
            '#!/bin/bash\nprintf "%s\\n" "$*" >> "${FC01_BUNDLE_TEST_ACTIONS:?}"\n')
        (source/'new_untracked_helper.py').write_text('NEW_SOURCE=True\n')
        (source/'native/config').mkdir();(source/'native/config/flight_test.yaml').write_text('target: 1.5\n')
        (source/'ARM64_BUILD_EVIDENCE.json').write_text('{"old":true}')
        (source/'build-complete.json').write_text('{"old":true}')
        for name in ('OFFLINE_DEPLOYMENT.json','ACTIVATION_STATE.json','deployment-config.json'):
            (source/name).write_text('{"machine_specific":true}')
        (source/'HOST_VERIFICATION.json').write_text('{"arm64_hardware_tested":false}')
        (source/'dist').mkdir();(source/'dist/old-installer.sh').write_text('old package')
        (source/'native/.build').mkdir();(source/'native/.build/host.o').write_bytes(b'ELF old object')
        home=folder/'home';home.mkdir()
        actions=folder/'actions.log'
        env={**os.environ,'HOME':str(home),'FC01_BUNDLE_TEST_ACTIONS':str(actions)}
        env.pop('FC01_NAVIGATION_INSTALL_DIR',None)
        return source,home,actions,env

    def execute(self,installer,env,*args):
        return subprocess.run(['bash',str(installer),*args],env=env,capture_output=True,text=True)

    def archive(self,installer):
        payload=base64.b64decode(b''.join(installer.read_bytes().split(MARKER,1)[1].split()),validate=True)
        with zipfile.ZipFile(io.BytesIO(payload)) as archive:
            return {name:archive.read(name) for name in archive.namelist()}

    def test_current_checkout_sources_and_local_edit_are_exact(self):
        manifest=bundle.make_manifest(ROOT)
        expected={str(path.relative_to(ROOT)) for path in (ROOT/'native/src').rglob('*') if path.is_file()}
        self.assertTrue(expected<=manifest['source_sha256'].keys())
        self.assertTrue(expected<=manifest['payload_sha256'].keys())
        for name in expected:
            self.assertEqual(manifest['source_sha256'][name],hashlib.sha256((ROOT/name).read_bytes()).hexdigest())
        self.assertEqual(manifest['version'],'20261110.6')
        self.assertEqual(manifest['controller_build'],'FC01_RECOVERY_DIAGNOSTIC_V4_20261110')
        self.assertNotIn(bundle.MANIFEST,manifest['payload_sha256'])
        self.assertNotIn('ARM64_BUILD_EVIDENCE.json',manifest['payload_sha256'])

    def test_new_default_directory_preserves_previous_unpacked_release(self):
        with tempfile.TemporaryDirectory() as directory:
            folder=Path(directory);source,home,_,env=self.fixture(folder)
            previous=home/'FC01_OrangePi_Navigation_20261110_vertical_diagnostic'
            previous.mkdir(mode=0o700)
            preserved=previous/'ACTIVATION_STATE.json';preserved.write_text('previous activation evidence\n')
            installer=bundle.build(folder/'installer.sh',source)
            result=self.execute(installer,env,'--extract-only')
            self.assertEqual(result.returncode,0,result.stdout+result.stderr)
            self.assertNotEqual(bundle.DEFAULT_DIRECTORY,previous.name)
            self.assertTrue((home/bundle.DEFAULT_DIRECTORY/'navigation.sh').is_file())
            self.assertEqual(preserved.read_text(),'previous activation evidence\n')
            files=self.archive(installer)
            release=json.loads(files[bundle.MANIFEST])
            self.assertEqual(release['release_name'],bundle.DEFAULT_DIRECTORY)
            header=installer.read_bytes().split(MARKER,1)[0]
            self.assertNotIn(b'__RELEASE_VERSION__',header)
            self.assertNotIn(b'__CONTROLLER_BUILD__',header)

    def test_default_offline_repeat_skips_unpack_and_preserves_generated_outputs(self):
        with tempfile.TemporaryDirectory() as directory:
            folder=Path(directory);source,home,actions,env=self.fixture(folder)
            installer=bundle.build(folder/'installer.sh',source,directory_name=bundle.RELEASE_NAME)
            first=self.execute(installer,env)
            self.assertEqual(first.returncode,0,first.stdout+first.stderr)
            self.assertIn('FC01_PAYLOAD_INSTALLED',first.stdout)
            target=home/bundle.RELEASE_NAME
            self.assertEqual(actions.read_text().splitlines(),['offline'])
            file=target/'native/src/jikl_fc01_system/src/fc01_config_check.cpp';stamp=file.stat().st_mtime_ns
            (target/'install').mkdir();(target/'install/generated').write_text('real build output')
            second=self.execute(installer,env,'offline')
            self.assertEqual(second.returncode,0,second.stdout+second.stderr)
            self.assertIn('FC01_PAYLOAD_VERIFIED_REUSED',second.stdout)
            self.assertEqual(file.stat().st_mtime_ns,stamp)
            self.assertEqual((target/'install/generated').read_text(),'real build output')
            self.assertEqual(actions.read_text().splitlines(),['offline','offline'])
            release=json.loads((target/bundle.MANIFEST).read_text())
            for name,expected in release['payload_sha256'].items():
                self.assertEqual(hashlib.sha256((target/name).read_bytes()).hexdigest(),expected)
            self.assertIn('new_untracked_helper.py',release['payload_sha256'])
            self.assertIn('HOST_VERIFICATION.json',release['payload_sha256'])
            for excluded in ('ARM64_BUILD_EVIDENCE.json','build-complete.json','OFFLINE_DEPLOYMENT.json',
                             'ACTIVATION_STATE.json','deployment-config.json','dist/old-installer.sh','native/.build/host.o'):
                self.assertNotIn(excluded,release['payload_sha256'])
            self.assertTrue(list((home/'FC01_navigation/deploy-logs'/bundle.RELEASE_NAME).glob('*extract.log')))

    def test_extract_only_compatibility_and_action_forwarding(self):
        with tempfile.TemporaryDirectory() as directory:
            folder=Path(directory);source,home,actions,env=self.fixture(folder)
            installer=bundle.build(folder/'installer.sh',source,directory_name=bundle.RELEASE_NAME)
            for arg in ('--install-only','extract-only','--extract-only'):
                result=self.execute(installer,env,arg)
                self.assertEqual(result.returncode,0,result.stdout+result.stderr)
                self.assertFalse(actions.exists())
            for args in (('activate','--confirm-flight-test'),('rollback','--backup','/tmp/review-backup')):
                result=self.execute(installer,env,*args)
                self.assertEqual(result.returncode,0,result.stdout+result.stderr)
            self.assertEqual(actions.read_text().splitlines(),['activate --confirm-flight-test','rollback --backup /tmp/review-backup'])
            result=self.execute(installer,env,'--install-only','offline')
            self.assertNotEqual(result.returncode,0)

    def test_manifest_checks_actual_disk_and_rejects_stale_content(self):
        with tempfile.TemporaryDirectory() as directory:
            folder=Path(directory);source,_,_,_=self.fixture(folder)
            manifest=bundle.make_manifest(source)
            (source/bundle.MANIFEST).write_text(json.dumps(manifest))
            installer=bundle.build(folder/'installer.sh',source)
            files=self.archive(installer)
            self.assertEqual(json.loads(files[bundle.MANIFEST]),manifest)
            local=source/'native/src/jikl_fc01_system/src/fc01_config_check.cpp'
            local.write_text('another uncommitted config-check change\n')
            with self.assertRaisesRegex(ValueError,'payload_sha256'):
                bundle.build(folder/'stale.sh',source)
            self.assertEqual(local.read_text(),'another uncommitted config-check change\n')

    def test_existing_different_or_incomplete_directory_is_never_overwritten(self):
        for condition in ('changed','missing','empty'):
            with self.subTest(condition=condition),tempfile.TemporaryDirectory() as directory:
                folder=Path(directory);source,home,actions,env=self.fixture(folder)
                installer=bundle.build(folder/'installer.sh',source,directory_name=bundle.RELEASE_NAME)
                target=home/bundle.RELEASE_NAME
                if condition=='empty':target.mkdir(mode=0o700)
                else:
                    self.assertEqual(self.execute(installer,env,'--install-only').returncode,0)
                    path=target/'navigation.sh'
                    if condition=='changed':path.write_text('keep user file')
                    else:path.unlink()
                result=self.execute(installer,env,'offline')
                self.assertNotEqual(result.returncode,0)
                self.assertIn('未覆盖',result.stderr)
                self.assertFalse(actions.exists())
                if condition=='changed':self.assertEqual((target/'navigation.sh').read_text(),'keep user file')
                if condition=='missing':self.assertFalse((target/'navigation.sh').exists())

    def test_whole_package_sidecar_and_archive_hash_fail_before_extract(self):
        for kind in ('header','payload','sidecar'):
            with self.subTest(kind=kind),tempfile.TemporaryDirectory() as directory:
                folder=Path(directory);source,home,_,env=self.fixture(folder)
                installer=bundle.build(folder/'installer.sh',source,directory_name=bundle.RELEASE_NAME)
                if kind=='header':installer.write_bytes(installer.read_bytes().replace(b'# Source-only release.',b'# Modified release.   ',1))
                elif kind=='sidecar':Path(str(installer)+'.sha256').write_text('0'*64+'  installer.sh\n')
                else:
                    header,encoded=installer.read_bytes().split(MARKER,1)
                    payload=bytearray(base64.b64decode(encoded));payload[-1]^=1
                    installer.write_bytes(header+MARKER+base64.encodebytes(payload))
                    Path(str(installer)+'.sha256').unlink()
                result=self.execute(installer,env,'--install-only')
                self.assertNotEqual(result.returncode,0)
                self.assertIn('SHA256',result.stderr)
                self.assertIn('log=',result.stderr)
                self.assertFalse((home/bundle.RELEASE_NAME).exists())

    def test_no_sidecar_uses_embedded_hash_and_rebuild_is_reproducible(self):
        with tempfile.TemporaryDirectory() as directory:
            folder=Path(directory);source,home,_,env=self.fixture(folder)
            installer=bundle.build(folder/'installer.sh',source,directory_name=bundle.RELEASE_NAME)
            before=installer.read_bytes();bundle.build(installer,source,directory_name=bundle.RELEASE_NAME)
            self.assertEqual(installer.read_bytes(),before)
            digest=hashlib.sha256(before).hexdigest()
            self.assertEqual(Path(str(installer)+'.sha256').read_text(),digest+'  installer.sh\n')
            Path(str(installer)+'.sha256').unlink()
            result=self.execute(installer,env,'--install-only')
            self.assertEqual(result.returncode,0,result.stdout+result.stderr)
            self.assertTrue((home/bundle.RELEASE_NAME).is_dir())

    def test_unsafe_archive_paths_are_rejected_before_creating_target(self):
        for name,mode in (('../escape',0o100600),('/absolute',0o100600),('link',stat.S_IFLNK|0o777)):
            with self.subTest(name=name),tempfile.TemporaryDirectory() as directory:
                folder=Path(directory);_,home,_,env=self.fixture(folder)
                stream=io.BytesIO()
                with zipfile.ZipFile(stream,'w') as archive:
                    info=zipfile.ZipInfo(name);info.create_system=3;info.external_attr=mode<<16
                    archive.writestr(info,b'unsafe')
                payload=stream.getvalue()
                header=(bundle.HEADER.replace('__ARCHIVE_SHA256__',hashlib.sha256(payload).hexdigest())
                        .replace('__DIRECTORY_NAME__',bundle.RELEASE_NAME).replace('__RELEASE_NAME__',bundle.RELEASE_NAME))
                installer=folder/'malicious.sh';installer.write_bytes(header.encode()+base64.encodebytes(payload))
                result=self.execute(installer,env,'--install-only')
                self.assertNotEqual(result.returncode,0)
                self.assertIn('不安全的安装项',result.stderr)
                self.assertFalse((home/bundle.RELEASE_NAME).exists())
                self.assertFalse((home/'escape').exists())


if __name__=='__main__':unittest.main()
