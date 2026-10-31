import copy
import importlib.util
import json
from pathlib import Path
import sys
import unittest

import yaml

ROOT = Path(__file__).resolve().parents[1]


def load_api(test):
    path = ROOT / 'fc01_control.py'
    test.assertTrue(path.is_file(), 'control entry is not implemented yet')
    spec = importlib.util.spec_from_file_location('fc01_control', path)
    api = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = api
    spec.loader.exec_module(api)
    return api


def ground():
    return {
        'kind': 'state', 'phase': 'IDLE', 'nav_state': 2,
        'armed': False, 'landed': True, 'at_rest': True,
        'status_fresh': True, 'local_fresh': True, 'land_fresh': True,
        'attitude_fresh': True, 'failsafe_fresh': True, 'battery_fresh': True,
        'local_sample_fresh': True, 'attitude_sample_fresh': True,
        'source_clock_valid': True, 'source_clock_validated': True,
        'z_valid': True, 'vz_valid': True, 'arming_state_known': True,
        'battery_connected': True, 'preflight': True,
        'rc_link_ready': True, 'rc_switches_ready': True,
        'rc_source_clock_valid': True, 'rc_switch_clock_valid': True,
        'rc_sticks_moving': False, 'rc_kill_latched': False,
        'publication_fault_latched': False, 'operator_takeover_latched': False,
        'stationary_seconds': 3.0, 'required_ground_seconds': 2.0,
        'source_counts': [1, 1, 1], 'authority_mode': 'UNSET',
        'authority_ready': False, 'authority_locked': False,
        'authority_attempt_mode': 'UNSET', 'm1_176_ack_accepted': False,
        'm1_176_ack_policy': 'R632_ACK176_STATE_CANDIDATE_V1',
        'ground_vz_ned': -0.004, 'ground_vz_tolerance': 0.1,
    }


def ready():
    s = ground()
    s.update(phase='OFFBOARD_READY', nav_state=14,
             authority_mode='RC_TAKEOVER', authority_ready=True,
             authority_locked=True, authority_attempt_mode='RC_TAKEOVER',
             m1_176_ack_accepted=True)
    return s


class ProfileTests(unittest.TestCase):
    def setUp(self):
        self.api = load_api(self)
        self.base = yaml.safe_load((ROOT/'tests/fixtures/base.yaml').read_text())

    def test_default_cannot_enable_actions_even_if_input_flags_are_true(self):
        p = self.base['fc01_system']['ros__parameters']
        p.update(flight_enabled=True, arm_enabled=True, takeoff_enabled=True,
                 autonomous_failsafe_confirmed=True, bench_arm_only_enabled=True)
        out = self.api.runtime_profile(self.base, False)['fc01_system']['ros__parameters']
        for key in ('flight_enabled', 'flight_actions_enabled', 'arm_enabled',
                    'takeoff_enabled', 'flight_parameters_confirmed',
                    'rc_takeover_confirmed', 'autonomous_failsafe_confirmed',
                    'bench_arm_only_enabled'):
            self.assertIs(out[key], False)

    def test_flight_uses_session_copy_preserves_numeric_parameters(self):
        before = copy.deepcopy(self.base)
        out = self.api.runtime_profile(self.base, True)['fc01_system']['ros__parameters']
        self.assertEqual(self.base, before)
        self.assertIs(out['flight_enabled'], False)
        for key in ('flight_actions_enabled', 'arm_enabled',
                    'takeoff_enabled', 'flight_parameters_confirmed', 'rc_takeover_confirmed'):
            self.assertIs(out[key], True)
        self.assertIs(out['autonomous_failsafe_confirmed'], False)
        self.assertIs(out['bench_arm_only_enabled'], False)
        self.assertIs(out['px4_land_fallback_confirmed'], False)
        self.assertEqual(out['aa_hover_thrust'], 0.475)
        self.assertEqual(out['aa_takeoff_reserve_thrust'], 0.04)
        self.assertEqual(out['aa_max_height'], 5.0)
        self.assertEqual(out['aa_max_flight_seconds'], 30.0)
        self.assertEqual(out['aa_max_tilt_deg'], 10.0)
        self.assertEqual(out['aa_land_sample_max_gap'], 1.25)

    def test_incompatible_profile_or_sim_clock_refused(self):
        for key, value in [('profile', 'flight'), ('use_sim_time', True),
                           ('aa_max_height', 6.0), ('aa_hover_thrust', float('nan')),
                           ('aa_land_sample_max_gap', 0.5)]:
            base = copy.deepcopy(self.base)
            base['fc01_system']['ros__parameters'][key] = value
            with self.subTest(key=key), self.assertRaises(self.api.ControlError):
                self.api.runtime_profile(base, True)


class MissionTests(unittest.TestCase):
    def setUp(self):
        self.api = load_api(self)
        self.m = self.api.Mission(5.0)

    def send(self, seq, op, at):
        self.m.sent(seq, op)
        self.m.reply({'kind': 'reply', 'seq': seq, 'accepted': True,
                      'message': op+'_ACCEPTED'}, at)

    def prepare(self):
        self.m.observe(ground(), 1.0)
        self.m.request_takeoff(0.5, 1.0)
        self.assertEqual(self.m.poll(1.0), ('AUTHORITY_RC', []))
        self.send(11, 'AUTHORITY_RC', 1.01)
        self.assertEqual(self.m.poll(1.02), ('ATTITUDE_READY', []))
        self.send(12, 'ATTITUDE_READY', 1.03)

    def test_start_never_requests_motion_without_operator_command(self):
        self.m.observe(ready(), 1.0)
        self.assertIsNone(self.m.poll(1.1))
        self.assertFalse(self.m.submitted)

    def test_exact_preparation_then_single_takeoff(self):
        self.prepare()
        self.assertIsNone(self.m.poll(1.04))
        self.m.observe(ready(), 1.1)
        self.assertEqual(self.m.poll(1.1), ('TAKEOFF', [0.5]))
        self.assertTrue(self.m.submitted)
        self.assertIsNone(self.m.poll(1.11))
        self.send(13, 'TAKEOFF', 1.12)
        self.assertIsNone(self.m.poll(2.0))
        with self.assertRaises(self.api.ControlError):
            self.m.request_takeoff(0.6, 2.0)

    def test_wrong_or_unsent_reply_does_not_advance_plan(self):
        self.m.observe(ground(), 1.0)
        self.m.request_takeoff(0.5, 1.0)
        self.m.poll(1.0)
        self.m.sent(4, 'AUTHORITY_RC')
        self.m.reply({'kind': 'reply', 'seq': 99, 'accepted': True}, 1.1)
        self.assertIsNone(self.m.poll(1.2))
        self.m.reply({'kind': 'state', 'seq': 4, 'accepted': True}, 1.3)
        self.assertIsNone(self.m.poll(1.4))

    def test_rejected_preparation_never_submits_takeoff(self):
        self.m.observe(ground(), 1.0)
        self.m.request_takeoff(0.5, 1.0)
        self.m.poll(1.0)
        self.m.sent(4, 'AUTHORITY_RC')
        self.m.reply({'kind': 'reply', 'seq': 4, 'accepted': False,
                      'message': 'AUTHORITY_SAFE_GROUND_REQUIRED'}, 1.1)
        self.m.observe(ready(), 1.2)
        self.assertIsNone(self.m.poll(1.2))
        self.assertFalse(self.m.submitted)
        self.assertIn('AUTHORITY_SAFE_GROUND_REQUIRED', self.m.error)

    def test_stale_state_cannot_request_takeoff(self):
        self.prepare()
        self.m.observe(ready(), 1.1)
        self.assertIsNone(self.m.poll(3.0))
        self.assertFalse(self.m.submitted)

    def test_candidate_ack_or_unready_authority_cannot_request_takeoff(self):
        for key, value in [('m1_176_ack_accepted', False), ('authority_ready', False),
                           ('authority_locked', False), ('armed', True)]:
            with self.subTest(key=key):
                self.m = self.api.Mission(5.0)
                self.prepare()
                s = ready(); s[key] = value
                self.m.observe(s, 1.1)
                self.assertIsNone(self.m.poll(1.1))
                self.assertFalse(self.m.submitted)

    def test_cancel_during_preparation_cannot_later_arm(self):
        self.prepare()
        self.m.cancel()
        self.m.observe(ready(), 1.1)
        self.assertIsNone(self.m.poll(1.2))
        self.assertFalse(self.m.submitted)

    def test_preparation_timeout_requests_only_ground_release_once(self):
        self.prepare()
        self.assertEqual(self.m.poll(28.0), ('LAND', []))
        self.assertIsNone(self.m.poll(29.0))
        self.assertFalse(self.m.submitted)

    def test_kill_or_takeover_never_advances_takeoff(self):
        for key in ('rc_kill_latched', 'operator_takeover_latched', 'publication_fault_latched'):
            with self.subTest(key=key):
                self.m = self.api.Mission(5.0)
                self.prepare()
                s = ready(); s[key] = True
                self.m.observe(s, 1.1)
                self.assertIsNone(self.m.poll(1.2))
                self.assertFalse(self.m.submitted)

    def test_invalid_height_never_queues_actions(self):
        for h in (0, -1, 5, 6, float('nan'), float('inf')):
            with self.subTest(height=h), self.assertRaises(self.api.ControlError):
                self.m.request_takeoff(h, 1.0)
        self.assertIsNone(self.m.poll(1.1))

    def test_no_rc_or_clock_evidence_means_no_preparation(self):
        for key in ('rc_link_ready', 'rc_switches_ready', 'source_clock_valid',
                    'attitude_sample_fresh', 'battery_connected'):
            with self.subTest(key=key):
                m = self.api.Mission(5.0)
                s = ground(); s[key] = False
                m.observe(s, 1.0); m.request_takeoff(0.5, 1.0)
                self.assertIsNone(m.poll(1.1))

    def test_gps_denied_xy_flags_do_not_replace_vertical_health(self):
        s = ground(); s.update(xy_valid=False, vxy_valid=False, estimate_valid=False)
        self.assertEqual(self.api.readiness_issues(s), [])


if __name__ == '__main__':
    unittest.main()
