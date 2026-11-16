"""发行证据不能凭空/部分绑定真实产物；不执行飞控。"""
import copy
from pathlib import Path
import tempfile
import unittest

from build_arm64_bundle import copy_install, sha, validate_evidence


class BundleEvidenceTest(unittest.TestCase):
    def fixture(self, root):
        paths = ['native/src/jikl_fc01_system/CMakeLists.txt',
                 'native/src/jikl_fc01_system/src/fc01_system_node.cpp',
                 *['native/tools/' + n for n in ('vision_odometry_adapter.py',
                    'velocity_command_diagnostic.py', 'vision_state_validation.py', 'vision_resource_budget.py')],
                 'install/lib/jikl_fc01_system/fc01_system_node',
                 'install/lib/jikl_fc01_system/fc01_config_check']
        for n in paths:
            p = root / n
            p.parent.mkdir(parents=True, exist_ok=True)
            p.write_text(n)
        return dict(control_candidate_commit='a'*40, architecture='aarch64', ubuntu='22.04', ros_distro='humble',
                    native_source_sha256={n: sha(root/n) for n in paths if n.startswith('native/')},
                    install_sha256={n[8:]: sha(root/n) for n in paths if n.startswith('install/')})

    def test_complete_evidence_accepts_and_partial_or_changed_rejects(self):
        with tempfile.TemporaryDirectory() as t:
            root = Path(t)
            evidence = self.fixture(root)
            validate_evidence(root, evidence, 'a'*40)
            for field in ('native_source_sha256', 'install_sha256'):
                bad = copy.deepcopy(evidence)
                bad[field].pop(next(iter(bad[field])))
                with self.assertRaisesRegex(ValueError, 'coverage'):
                    validate_evidence(root, bad, 'a'*40)
            (root/'install/lib/jikl_fc01_system/fc01_system_node').write_text('different binary')
            with self.assertRaisesRegex(ValueError, 'bytes differ'):
                validate_evidence(root, evidence, 'a'*40)

    def test_install_directory_link_is_explicitly_rejected(self):
        with tempfile.TemporaryDirectory() as t:
            root = Path(t)
            source = root/'install'
            source.mkdir()
            (source/'ordinary').mkdir()
            (source/'alias').symlink_to(source/'ordinary', target_is_directory=True)
            with self.assertRaisesRegex(ValueError, 'directory symlink'):
                copy_install(source, root/'output')


if __name__ == '__main__':
    unittest.main()
