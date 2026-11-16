"""Prebuilt entry contracts using temporary payloads; no ROS or hardware starts."""
import copy
import hashlib
import json
import os
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest.mock import patch

import launcher
import navigation_prebuilt_service as service
import navigation_service
import yaml


class Fixture:
    def __init__(self, root):
        self.home = Path(root)
        self.entry = self.home / 'release'
        self.entry.mkdir()
        self.source = self.entry / 'deployment/runtime.yaml'
        self.source.parent.mkdir()
        self.original = {
            'another_node': {'ros__parameters': {'keep': [1, 2, {'value': True}]}},
            'fc01_system': {'ros__parameters': {
                'profile': 'attitude_altitude', 'use_sim_time': False,
                'flight_enabled': False, 'flight_actions_enabled': True,
                'arm_enabled': False, 'takeoff_enabled': True,
                'flight_parameters_confirmed': True, 'rc_takeover_confirmed': True,
                'autonomous_failsafe_confirmed': False, 'bench_arm_only_enabled': False,
                'visual_navigation_enabled': False, 'rc_navigation_enabled': True,
                'rc_navigation_mode_slot': 4, 'rc_target_height': 1.75,
                'aa_max_height': 3.25, 'aa_hover_thrust': 0.31,
                'aa_takeoff_reserve_thrust': 0.028, 'aa_max_tilt_deg': 8.5,
                'csv_path': '/original/custom.csv', 'unknown_future_key': [4, 5],
                'fc01_entry_version': 'old', 'fc01_config_path': '/old/runtime.yaml',
                'fc01_config_source_path': '/old/source.yaml',
            }},
        }
        self.source.write_text(yaml.safe_dump(self.original))
        self.environment = {
            'sdk_root': '/old/sdk', 'agent_path': '/original/MicroXRCEAgent',
            'mode': 'flight-test', 'environment': {
                'ROS_DOMAIN_ID': '0', 'ROS_LOCALHOST_ONLY': '0',
                'RMW_IMPLEMENTATION': 'rmw_fastrtps_cpp',
                'FASTRTPS_DEFAULT_PROFILES_FILE': '/original/dds.xml',
                'PYTHONPATH': '/old/px4_msgs', 'LD_LIBRARY_PATH': '/old/lib',
                'HOME': '/untrusted', 'PATH': '/old/bin',
            },
        }
        self.write_environment()
        elf = bytearray(64)
        elf[:6] = b'\x7fELF\x02\x01'
        elf[18:20] = (183).to_bytes(2, 'little')
        markers = (launcher.BINARY_MARKERS + (
            navigation_service.MARKER, navigation_service.TAKEOFF_MARKER,
            navigation_service.SAFETY_MARKER, navigation_service.LIFECYCLE_MARKER,
        ))
        self.binary = self.entry / service.CONTROLLER
        self.binary.parent.mkdir(parents=True)
        self.binary.write_bytes(elf + b' '.join(markers))
        self.binary.chmod(0o700)
        self.checker = self.binary.parent / 'fc01_config_check'
        self.checker.write_bytes(elf)
        self.checker.chmod(0o700)
        for name in ('navigation_prebuilt_service.py', 'prebuilt_boot.sh', 'install/local_setup.bash'):
            path = self.entry / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text('# immutable fixture\n')
        self.manifest = dict(
            version='20261116.stage3-abcdef', source_commit='a' * 40,
            architecture='aarch64', ubuntu='22.04', ros_distro='humble',
            controller_relative=service.CONTROLLER,
        )
        self.write_manifest()
        self.calls = []

    def write_environment(self):
        (self.entry / 'deployment/environment.json').write_text(json.dumps(self.environment))

    def write_manifest(self):
        self.manifest['payload_sha256'] = {
            str(path.relative_to(self.entry)): hashlib.sha256(path.read_bytes()).hexdigest()
            for path in self.entry.rglob('*') if path.is_file()
            and not path.is_relative_to(self.entry / 'deployment')
            and path.name != 'ARM64_RELEASE.json'
        }
        (self.entry / 'ARM64_RELEASE.json').write_text(json.dumps(self.manifest))

    def run(self, argv, **kwargs):
        self.calls.append(argv)
        if argv[0] == str(self.checker):
            checked = yaml.safe_load(Path(argv[-1]).read_text())
            self.assert_preserved(checked)
        return subprocess.CompletedProcess(argv, 0, stdout='validated\n', stderr='')

    def assert_preserved(self, profile):
        expected = copy.deepcopy(self.original)
        expected['fc01_system']['ros__parameters'].update(
            fc01_entry_version=self.manifest['version'],
            fc01_config_path=str(self.home / 'FC01_navigation/runtime.yaml'),
            fc01_config_source_path=str(self.source),
        )
        if profile != expected:
            raise AssertionError((profile, expected))


class PrebuiltServiceTests(unittest.TestCase):
    def test_snapshot_preserves_every_control_and_only_changes_identity(self):
        with tempfile.TemporaryDirectory() as root:
            fixture = Fixture(root)
            profile = service.snapshot_profile(fixture.entry, fixture.manifest, home=fixture.home)
            fixture.assert_preserved(profile)
            self.assertEqual(yaml.safe_load(fixture.source.read_text()), fixture.original)
            profile['another_node']['ros__parameters']['keep'].append(9)
            self.assertEqual(yaml.safe_load(fixture.source.read_text()), fixture.original)

    def test_inherited_settings_refuse_incompatible_policy_without_mutation(self):
        changes = (
            {'arm_enabled': True}, {'visual_navigation_enabled': True},
            {'flight_enabled': True}, {'rc_navigation_enabled': False},
            {'rc_target_height': float('nan')}, {'rc_navigation_mode_slot': True},
        )
        with tempfile.TemporaryDirectory() as root:
            fixture = Fixture(root)
            for change in changes:
                with self.subTest(change=change):
                    modified = copy.deepcopy(fixture.original)
                    modified['fc01_system']['ros__parameters'].update(change)
                    fixture.source.write_text(yaml.safe_dump(modified))
                    before = fixture.source.read_bytes()
                    with self.assertRaises(service.ControlError):
                        service.snapshot_profile(fixture.entry, fixture.manifest)
                    self.assertEqual(fixture.source.read_bytes(), before)

    def test_environment_preserves_dds_and_discards_old_overlay_and_reserved_paths(self):
        with tempfile.TemporaryDirectory() as root:
            fixture = Fixture(root)
            loaded = service.deployment_environment(fixture.entry)
            self.assertEqual(loaded['agent_path'], '/original/MicroXRCEAgent')
            self.assertEqual(loaded['environment'], {
                'ROS_DOMAIN_ID': '0', 'ROS_LOCALHOST_ONLY': '0',
                'RMW_IMPLEMENTATION': 'rmw_fastrtps_cpp',
                'FASTRTPS_DEFAULT_PROFILES_FILE': '/original/dds.xml',
            })
            for key, value in (('ROS_DOMAIN_ID', '4'), ('ROS_LOCALHOST_ONLY', '1'),
                               ('FC01_BAUD', '921600'), ('FC01_DEVICE', '/dev/ttyS0')):
                with self.subTest(key=key):
                    fixture.environment['environment'][key] = value
                    fixture.write_environment()
                    with self.assertRaises(service.ControlError):
                        service.deployment_environment(fixture.entry)
                    fixture.environment['environment'].pop(key)

    def test_check_runs_only_ldd_and_packaged_checker_with_preserved_config(self):
        with tempfile.TemporaryDirectory() as root:
            fixture = Fixture(root)
            with (patch.object(service.platform, 'machine', return_value='aarch64'),
                  patch.dict(os.environ, ROS_DISTRO='humble', ROS_DOMAIN_ID='0',
                             ROS_LOCALHOST_ONLY='0'),
                  patch.object(Path, 'home', return_value=fixture.home),
                  patch.object(service, 'check_type_support') as types,
                  patch.object(service.standby, 'supervise') as supervise):
                manifest, profile = service.check(fixture.entry, run=fixture.run)
            fixture.assert_preserved(profile)
            self.assertEqual(manifest, fixture.manifest)
            types.assert_called_once_with(fixture.entry)
            supervise.assert_not_called()
            self.assertEqual([call[0] for call in fixture.calls],
                             ['/usr/bin/ldd', '/usr/bin/ldd', str(fixture.checker)])
            self.assertNotIn(str(fixture.binary), [call[0] for call in fixture.calls])

    def test_payload_hash_arch_and_dependency_errors_refuse_before_checker(self):
        with tempfile.TemporaryDirectory() as root:
            fixture = Fixture(root)
            with (patch.object(service.platform, 'machine', return_value='aarch64'),
                  patch.dict(os.environ, ROS_DISTRO='humble', ROS_DOMAIN_ID='0',
                             ROS_LOCALHOST_ONLY='0'),
                  patch.object(service, 'check_type_support')):
                fixture.binary.write_bytes(fixture.binary.read_bytes() + b'tampered')
                with self.assertRaisesRegex(service.ControlError, 'hash/path'):
                    service.check(fixture.entry, run=fixture.run)
                self.assertEqual(fixture.calls, [])
                data = bytearray(fixture.binary.read_bytes())
                data[18:20] = (62).to_bytes(2, 'little')
                fixture.binary.write_bytes(data)
                fixture.write_manifest()
                with self.assertRaisesRegex(service.ControlError, 'ARM64'):
                    service.check(fixture.entry, run=fixture.run)
                self.assertEqual(fixture.calls, [])
                data[18:20] = (183).to_bytes(2, 'little')
                fixture.binary.write_bytes(data)
                fixture.write_manifest()

                def missing_library(argv, **kwargs):
                    fixture.calls.append(argv)
                    return subprocess.CompletedProcess(argv, 0, 'libbad.so => not found\n', '')

                with self.assertRaisesRegex(service.ControlError, 'not found'):
                    service.check(fixture.entry, run=missing_library)
                self.assertEqual(len(fixture.calls), 1)

    def test_supervisor_uses_packaged_node_snapshot_and_original_agent(self):
        with tempfile.TemporaryDirectory() as root:
            fixture = Fixture(root)
            profile = service.snapshot_profile(fixture.entry, fixture.manifest, home=fixture.home)
            with (patch.object(service, 'check', return_value=(fixture.manifest, profile)),
                  patch.object(Path, 'home', return_value=fixture.home),
                  patch.object(service.signal, 'signal') as signals,
                  patch.object(service.standby, 'supervise') as supervise):
                service.run_service(fixture.entry)
            args, options = supervise.call_args
            self.assertEqual(args[:3], (fixture.entry, Path('/original/MicroXRCEAgent'),
                                       fixture.home / 'FC01_navigation'))
            self.assertEqual(options['controller'], fixture.binary)
            self.assertEqual(options['config_path'], fixture.source)
            self.assertIs(options['row_formatter'], navigation_service.describe_row)
            with patch.object(Path, 'home', return_value=fixture.home):
                fixture.assert_preserved(
                    options['config_factory'](Path('/wrong/sdk'), Path('/wrong/agent'))
                )
            metadata = options['status_metadata']
            self.assertEqual(metadata['entry_version'], fixture.manifest['version'])
            self.assertEqual(metadata['source_commit'], fixture.manifest['source_commit'])
            self.assertEqual(metadata['rc_target_height'], 1.75)
            self.assertFalse(metadata['automatic_arm'])
            for call in signals.call_args_list:
                call.args[1](None, None)
            self.assertTrue(args[3].is_set())
            fixture.source.write_text(fixture.source.read_text() + '# changed\n')
            with self.assertRaisesRegex(service.ControlError, 'snapshot changed'):
                options['config_factory'](fixture.entry, Path('/original/MicroXRCEAgent'))


if __name__ == '__main__':
    unittest.main()
