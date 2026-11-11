"""部署证据的临时资源归属回归；外部构建和动态库查询全部使用替身。"""

from contextlib import ExitStack
import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import navigation_build_evidence as evidence
from test_navigation_build_evidence import BuildFixture


class TemporaryEvidenceOwnershipTests(unittest.TestCase):
    """先存在的临时文件属于先前操作，创建失败不能把它当作本轮垃圾删除。"""

    def test_json_collision_preserves_existing_regular_file_and_symlink(self):
        for symlink in (False, True):
            with self.subTest(symlink=symlink), tempfile.TemporaryDirectory() as folder:
                destination = Path(folder) / 'evidence.json'
                destination.write_text('old complete evidence')
                temporary = Path(str(destination) + '.tmp-' + str(os.getpid()))
                target = Path(folder) / 'previous-record'
                target.write_text('previous temporary evidence')
                if symlink:
                    temporary.symlink_to(target)
                else:
                    temporary.write_bytes(target.read_bytes())
                with self.assertRaises(FileExistsError):
                    evidence.atomic_json(destination, {'new': True})
                self.assertEqual(temporary.read_text(), 'previous temporary evidence')
                self.assertEqual(temporary.is_symlink(), symlink)
                self.assertEqual(target.read_text(), 'previous temporary evidence')
                self.assertEqual(destination.read_text(), 'old complete evidence')

    def test_json_replace_failure_removes_owned_temporary_and_preserves_error(self):
        with tempfile.TemporaryDirectory() as folder:
            destination = Path(folder) / 'evidence.json'
            destination.write_text('old complete evidence')
            temporary = Path(str(destination) + '.tmp-' + str(os.getpid()))
            error = OSError('injected replace failure')
            with patch.object(evidence.os, 'replace', side_effect=error):
                with self.assertRaises(OSError) as caught:
                    evidence.atomic_json(destination, {'new': True})
            self.assertIs(caught.exception, error)
            self.assertFalse(temporary.exists())
            self.assertEqual(destination.read_text(), 'old complete evidence')

    def test_json_serialization_failure_removes_owned_temporary(self):
        with tempfile.TemporaryDirectory() as folder:
            destination = Path(folder) / 'evidence.json'
            temporary = Path(str(destination) + '.tmp-' + str(os.getpid()))
            with self.assertRaises(TypeError):
                evidence.atomic_json(destination, {'unsupported': object()})
            self.assertFalse(temporary.exists())
            self.assertFalse(destination.exists())


class TemporaryInstallLinkOwnershipTests(unittest.TestCase):
    """使用原有 fake CMake 夹具，仅在临时发布树里走真实 build_release 协调代码。"""

    def setUp(self):
        self.resources = ExitStack()
        self.addCleanup(self.resources.close)
        folder = self.resources.enter_context(tempfile.TemporaryDirectory())
        self.fixture = BuildFixture(folder)
        for replacement in self.fixture.patches():
            self.resources.enter_context(replacement)

    def build(self):
        return evidence.build_release(self.fixture.entry, self.fixture.sdk)

    def test_link_collision_preserves_existing_regular_file_and_symlink(self):
        first = self.build()
        saved = (self.fixture.entry / 'ARM64_BUILD_EVIDENCE.json').read_bytes()
        self.fixture.dependencies['trees'][0]['sha256'] = '2' * 64
        temporary = self.fixture.entry / ('.install-new-' + str(os.getpid()))
        target = self.fixture.entry / 'retained-link-target'
        target.mkdir()
        for symlink in (False, True):
            with self.subTest(symlink=symlink):
                if symlink:
                    temporary.symlink_to(target, target_is_directory=True)
                else:
                    temporary.write_text('previous operation')
                with self.assertRaises(FileExistsError):
                    self.build()
                self.assertEqual(temporary.is_symlink(), symlink)
                if symlink:
                    self.assertEqual(temporary.resolve(), target)
                else:
                    self.assertEqual(temporary.read_text(), 'previous operation')
                self.assertEqual(
                    (self.fixture.entry / 'install').resolve(), Path(first['install_prefix'])
                )
                self.assertEqual(
                    (self.fixture.entry / 'ARM64_BUILD_EVIDENCE.json').read_bytes(), saved
                )
                temporary.unlink()
        self.assertEqual(len(self.fixture.calls), 9)

    def test_link_replace_failure_removes_owned_link_and_preserves_error(self):
        first = self.build()
        saved = (self.fixture.entry / 'ARM64_BUILD_EVIDENCE.json').read_bytes()
        self.fixture.dependencies['trees'][0]['sha256'] = '3' * 64
        temporary = self.fixture.entry / ('.install-new-' + str(os.getpid()))
        error = OSError('injected install replace failure')
        with patch.object(evidence.os, 'replace', side_effect=error):
            with self.assertRaises(OSError) as caught:
                self.build()
        self.assertIs(caught.exception, error)
        self.assertFalse(temporary.exists())
        self.assertFalse(temporary.is_symlink())
        self.assertEqual((self.fixture.entry / 'install').resolve(), Path(first['install_prefix']))
        self.assertEqual((self.fixture.entry / 'ARM64_BUILD_EVIDENCE.json').read_bytes(), saved)
        self.assertEqual(json.loads(saved)['install_prefix'], first['install_prefix'])
        self.assertEqual(len(self.fixture.calls), 6)


if __name__ == '__main__':
    unittest.main()
