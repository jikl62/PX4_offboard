"""One initial platform height across installed and offline configuration paths."""
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

import yaml
import navigation_service
import launcher

ROOT = Path(__file__).resolve().parents[1]


class TakeoffConfigurationTests(unittest.TestCase):
    def test_attitude_altitude_defaults_load_new_thrust_parameters(self):
        """The shipped YAML is the source consumed by profile generation."""
        params = yaml.safe_load((ROOT/'native/config/attitude_altitude.yaml').read_text())[
            'fc01_system']['ros__parameters']
        self.assertEqual(params['aa_hover_thrust'], .50)
        self.assertEqual(params['aa_takeoff_reserve_thrust'], .04)
        self.assertEqual(params['aa_thrust_min'], .12)
        self.assertEqual(params['aa_thrust_max'], .80)
        self.assertEqual(params['aa_thrust_slew'], .40)

    def test_old_manual_navigation_binary_cannot_pass_new_takeoff_release_check(self):
        with tempfile.TemporaryDirectory() as folder, patch.object(launcher.platform,'machine',return_value='aarch64'):
            entry=Path(folder);binary=entry/'install/lib/jikl_fc01_system/fc01_system_node'
            binary.parent.mkdir(parents=True)
            data=bytearray(64);data[:6]=b'\x7fELF\x02\x01';data[18:20]=(183).to_bytes(2,'little')
            old=bytes(data)+b''.join(launcher.BINARY_MARKERS)+navigation_service.MARKER
            binary.write_bytes(old);binary.chmod(0o700)
            with self.assertRaises(navigation_service.ControlError):
                navigation_service.check_controller(entry)
            binary.write_bytes(old+b'NAV_TAKEOFF_EVIDENCE_V2_20261103')
            with self.assertRaises(navigation_service.ControlError):
                navigation_service.check_controller(entry)
            binary.write_bytes(old+navigation_service.TAKEOFF_MARKER+navigation_service.SAFETY_MARKER)
            with self.assertRaisesRegex(navigation_service.ControlError, '生命周期'):
                navigation_service.check_controller(entry)
            binary.write_bytes(old+navigation_service.TAKEOFF_MARKER+navigation_service.SAFETY_MARKER+
                               navigation_service.LIFECYCLE_MARKER)
            self.assertEqual(navigation_service.check_controller(entry),binary)

    def test_missing_settings_and_explicit_settings_share_platform_default(self):
        base = yaml.safe_load((ROOT/'native/config/attitude_altitude.yaml').read_text())
        with tempfile.TemporaryDirectory() as folder, patch.object(navigation_service.standby, 'validate', return_value=base):
            entry = Path(folder)
            for selected in (None, {}, {'navigation_mode_slot': 1}, {'target_height_m': 1.5}):
                path = entry/'navigation_settings.json'
                if selected is not None:
                    path.write_text(json.dumps(selected))
                profile = navigation_service.load_profile(entry, entry/'agent', entry)
                p = profile['fc01_system']['ros__parameters']
                self.assertEqual(p['rc_target_height'], 1.5)
                self.assertFalse(p['arm_enabled'])
                for key, value in base['fc01_system']['ros__parameters'].items():
                    if key.startswith('aa_'):
                        self.assertEqual(p[key], value, key)
            # An explicit task setting remains configurable; no global clamp to 1.5.
            path.write_text(json.dumps({'target_height_m': 2.0}))
            self.assertEqual(navigation_service.load_profile(entry, entry/'agent', entry)
                             ['fc01_system']['ros__parameters']['rc_target_height'], 2.0)

    def test_shipping_settings_and_offline_export_use_1p5m_without_changing_gains(self):
        self.assertEqual(json.loads((ROOT/'navigation_settings.json').read_text())['target_height_m'], 1.5)
        base = yaml.safe_load((ROOT/'native/config/attitude_altitude.yaml').read_text())['fc01_system']['ros__parameters']
        self.assertEqual(base['rc_target_height'], 1.5)
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/'runtime.yaml'
            for enabled in (False, True):
                args = [sys.executable, str(ROOT/'vision_config.py'), '--output', str(path)]
                if enabled:
                    args.append('--enable-vision')
                result = subprocess.run(args, capture_output=True, text=True, timeout=5)
                self.assertEqual(result.returncode, 0, result.stderr)
                p = yaml.safe_load(path.read_text())['fc01_system']['ros__parameters']
                self.assertEqual(p['rc_target_height'], 1.5)
                self.assertEqual(p['rc_navigation_mode_slot'], 1)
                self.assertIs(p['visual_navigation_enabled'], enabled)
                self.assertEqual(p['aa_hover_thrust'], .50)
                self.assertEqual(p['aa_thrust_max'], .80)
                self.assertEqual(p['aa_takeoff_reserve_thrust'], .04)
                self.assertEqual(p['aa_local_timeout'], .5)
                self.assertEqual(p['aa_feedback_rate_tolerance_mps'], .2)
                self.assertFalse(p['arm_enabled'])


if __name__ == '__main__':
    unittest.main()
