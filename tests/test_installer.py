import base64
import importlib
import io
import os
from pathlib import Path
import subprocess
import tempfile
import unittest
import zipfile

ROOT = Path(__file__).resolve().parents[1]


class InstallerTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(prefix='fc01_install_')
        self.addCleanup(self.tmp.cleanup)
        self.folder = Path(self.tmp.name)
        if not (ROOT/'build_bundle.py').is_file():
            raise AssertionError('offline self-extractor is not implemented yet')
        self.builder = importlib.import_module('build_bundle')
        self.script = self.folder/'control.sh'
        self.target = self.folder/'installed'

    def archive(self, files):
        stream = io.BytesIO()
        with zipfile.ZipFile(stream, 'w', zipfile.ZIP_DEFLATED) as z:
            for name, data in files.items():
                z.writestr(name, data)
        return stream.getvalue()

    def run_script(self, *args, stdin=None):
        env = {**os.environ, 'FC01_CONTROL_INSTALL_DIR': str(self.target)}
        return subprocess.run(['bash', str(self.script), *args], env=env, input=stdin,
                              capture_output=True, text=True, timeout=10)

    def build_actual(self):
        self.builder.build(self.script, ROOT)

    def test_real_bundle_installs_offline_without_launching_hardware(self):
        self.build_actual()
        p = self.run_script('--install-only')
        self.assertEqual(p.returncode, 0, p.stderr+p.stdout)
        for name in ('start.sh', 'fc01_control.py', 'launcher.py', 'README_zh.md', 'VERIFICATION.md'):
            self.assertEqual((self.target/name).read_bytes(), (ROOT/name).read_bytes())
        self.assertEqual(self.target.stat().st_mode & 0o777, 0o700)

    def test_repeat_install_preserves_equal_files_and_unrelated_data(self):
        self.build_actual()
        self.assertEqual(self.run_script('--install-only').returncode, 0)
        (self.target/'operator-notes.txt').write_text('keep me')
        self.assertEqual(self.run_script('--install-only').returncode, 0)
        self.assertEqual((self.target/'operator-notes.txt').read_text(), 'keep me')

    def test_existing_different_file_refused_before_any_payload_write(self):
        self.build_actual()
        self.target.mkdir()
        (self.target/'start.sh').write_text('operator version')
        p = self.run_script('--install-only')
        self.assertNotEqual(p.returncode, 0)
        self.assertEqual((self.target/'start.sh').read_text(), 'operator version')
        self.assertFalse((self.target/'fc01_control.py').exists())

    def test_existing_symlink_never_redirects_writes(self):
        self.build_actual()
        outside = self.folder/'outside'
        outside.mkdir()
        self.target.symlink_to(outside, target_is_directory=True)
        self.assertNotEqual(self.run_script('--install-only').returncode, 0)
        self.assertEqual(list(outside.iterdir()), [])

    def test_payload_checksum_failure_leaves_target_absent(self):
        self.build_actual()
        data = self.script.read_bytes()
        before, encoded = data.split(b'\n__FC01_PAYLOAD_BASE64__\n', 1)
        raw = bytearray(base64.b64decode(encoded))
        raw[len(raw)//2] ^= 1
        self.script.write_bytes(before+b'\n__FC01_PAYLOAD_BASE64__\n'+base64.encodebytes(raw))
        self.assertNotEqual(self.run_script('--install-only').returncode, 0)
        self.assertFalse(self.target.exists())

    def test_even_matching_hash_archive_cannot_escape_install_directory(self):
        for path in ('../escape', '/tmp/fc01_escape', 'a/../../escape'):
            with self.subTest(path=path):
                self.script.write_bytes(self.builder.render_archive(self.archive({'start.sh': '#!/bin/bash\n', path: 'no'})))
                self.assertNotEqual(self.run_script('--install-only').returncode, 0)
                self.assertFalse(self.target.exists())
                self.assertFalse((self.folder/'escape').exists())

    def test_archive_symlink_is_refused(self):
        stream = io.BytesIO()
        with zipfile.ZipFile(stream, 'w') as z:
            entry = zipfile.ZipInfo('start.sh')
            entry.create_system = 3
            entry.external_attr = 0o120777 << 16
            z.writestr(entry, '/tmp/elsewhere')
        self.script.write_bytes(self.builder.render_archive(stream.getvalue()))
        self.assertNotEqual(self.run_script('--install-only').returncode, 0)
        self.assertFalse(self.target.exists())

    def test_operator_stdin_survives_python_unpacking(self):
        script = '#!/usr/bin/env bash\nread -r fc01_line\nprintf "OPERATOR:%s\\n" "$fc01_line"\n'
        self.script.write_bytes(self.builder.render_archive(self.archive({'start.sh': script})))
        p = self.run_script(stdin='takeoff 0.5\n')
        self.assertEqual(p.returncode, 0, p.stderr+p.stdout)
        self.assertIn('OPERATOR:takeoff 0.5', p.stdout)


if __name__ == '__main__':
    unittest.main()
