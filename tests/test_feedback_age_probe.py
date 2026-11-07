"""Diagnostic arithmetic only; no ROS/Agent/serial needed."""
import argparse
import json
from pathlib import Path
import tempfile
import unittest

from feedback_age_probe import (StampProgress, TopicObservation, age_seconds,
                                duration_arg, read_controller_status, slope)


class FeedbackAgeProbeTests(unittest.TestCase):
    def test_missing_timestamp_and_rmw_zero_are_unavailable(self):
        observed = TopicObservation()
        observed.observe(0, 0, {'received_timestamp': 0}, 1_000_000, 10.)
        row = observed.report(1_000_010, 10.01, 5.)
        self.assertIsNone(row['last']['callback_envelope_age_s'])
        self.assertIsNone(row['last']['callback_sample_age_s'])
        self.assertIsNone(row['last']['rmw_to_callback_s'])
        self.assertEqual(row['last']['rmw_received_timestamp_ns'], 0)
        self.assertFalse(row['last']['rmw_received_available'])

    def test_signed_age_and_rmw_gap_preserve_future_values(self):
        observed = TopicObservation()
        observed.observe(1_100_000, 1_050_000, {'received_timestamp': 1_010_000_000}, 1_000_000, 10.)
        self.assertAlmostEqual(observed.last['callback_envelope_age_s'], -.1)
        self.assertAlmostEqual(observed.last['rmw_to_callback_s'], -.01)

    def test_new_envelope_cannot_refresh_frozen_old_raw(self):
        observed = TopicObservation()
        observed.observe(10_000_000, 9_400_000, {}, 10_000_000, 10.)
        observed.observe(10_050_000, 9_400_000, {}, 10_050_000, 10.05)
        self.assertEqual(observed.last['callback_envelope_age_s'], 0.)
        self.assertAlmostEqual(observed.last['callback_sample_age_s'], .65)
        self.assertEqual(observed.sample_progress.counts['duplicate'], 1)

    def test_sample_progress_records_duplicate_regression_and_missing(self):
        progress = StampProgress()
        self.assertEqual([progress.observe(x) for x in (10, 20, 20, 15, 0)],
                         ['first', 'advanced', 'duplicate', 'regressed', 'missing'])
        self.assertEqual(progress.counts, dict(first=1, advanced=1, duplicate=1, regressed=1, missing=1))

    def test_fixed_window_slope_and_expiry(self):
        observed = TopicObservation()
        for mono in (1., 2., 3.):
            host = 1_000_000_000+int(mono*1e6)
            envelope = host-int((.1+.002*mono)*1e6)
            observed.observe(envelope, envelope-2000, {}, host, mono)
        row = observed.report(host, 3., 3.)
        trend = row['envelope_age_trend']
        self.assertAlmostEqual(trend['slope_s_per_s'], .002)
        self.assertAlmostEqual(trend['r_squared'], 1.)
        expired = observed.report(host, 34., 31.)
        self.assertEqual(expired['envelope_age_trend']['samples'], 0)
        self.assertIsNone(expired['envelope_age_trend']['slope_s_per_s'])

    def test_wall_jump_changes_diagnostics_instead_of_correcting_source(self):
        observed = TopicObservation()
        observed.observe(1_000_000, 999_000, {}, 1_000_000, 1.)
        observed.observe(1_050_000, 1_049_000, {}, 2_050_000, 1.05)
        self.assertAlmostEqual(observed.last['callback_envelope_age_s'], 1.)
        self.assertEqual(observed.last['envelope_us'], 1_050_000)
        self.assertLess(age_seconds(1_000_000, 2_000_000), 0.)

    def test_read_only_controller_comparison_distinguishes_sample_instants(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'status.json'
            fixture = dict(updated_unix=10., feedback_current=False, state_controller_pid=23,
                           state=dict(diagnostic_host_us=10_000_000, source_timing=dict(local=dict(
                               envelope_us=9_800_000, sample_us=9_799_000, envelope_age_s=.2))))
            path.write_text(json.dumps(fixture))
            row = read_controller_status(path, 10_100_000, {'local': {'last': {'envelope_us': 10_000_000}}})
            self.assertTrue(row['available'])
            self.assertFalse(row['feedback_current'])
            self.assertFalse(row['topics']['local']['same_envelope'])
            self.assertAlmostEqual(row['topics']['local']['probe_minus_controller_envelope_s'], .2)
            self.assertEqual(json.loads(path.read_text()), fixture)

    def test_duration_is_bounded(self):
        self.assertEqual(duration_arg('5'), 5.)
        self.assertEqual(duration_arg('180'), 180.)
        for value in ('4', '181', 'nan', 'inf'):
            with self.assertRaises(argparse.ArgumentTypeError):
                duration_arg(value)

    def test_slope_needs_multiple_different_receipt_instants(self):
        self.assertIsNone(slope([])['slope_s_per_s'])
        self.assertIsNone(slope([(1., .2)])['slope_s_per_s'])
        self.assertIsNone(slope([(1., .2), (1., .3)])['slope_s_per_s'])


if __name__ == '__main__':
    unittest.main()
