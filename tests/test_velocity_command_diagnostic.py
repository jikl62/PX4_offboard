"""Host-only checks of the independent receiver and its bounded evidence files."""
import ast
import importlib.util
import json
import math
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace as NS
import unittest


SCRIPT = Path(__file__).resolve().parents[1] / 'native/tools/velocity_command_diagnostic.py'
SPEC = importlib.util.spec_from_file_location('fc01_velocity_diagnostic_test', SCRIPT)
diagnostic = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = diagnostic
SPEC.loader.exec_module(diagnostic)


def message(sequence=1, **changes):
    fields = dict(schema_version=1, sequence=sequence,
                  stamp=NS(sec=100, nanosec=sequence * 1000000),
                  reset_counter=0, source_id='synthetic-session-1', frame_id='body_frd',
                  valid=True, vx=.2, vy=-.1, vz=-.05)
    fields.update(changes)
    return NS(**fields)


class ReceiverTests(unittest.TestCase):
    def receiver(self, **limits):
        return diagnostic.Receiver(diagnostic.DiagnosticConfig(**limits))

    def test_defaults_record_data_without_claiming_clock_or_control_qualification(self):
        receiver = self.receiver()
        event = receiver.receive(message(), 10., 100001000000, 1)
        self.assertTrue(event['accepted'])
        state = receiver.snapshot(500., 900000000000, 1)
        self.assertFalse(state['control_authority'])
        self.assertEqual(state['qualification'], 'DIAGNOSTIC_ONLY')
        self.assertEqual(state['freshness'], 'UNCONFIGURED')
        self.assertFalse(state['cache_usable'])
        self.assertIsNone(state['current_candidate'])
        self.assertIn('SOURCE_BINDING_UNCONFIGURED', state['cache_errors'])
        self.assertTrue(all(value is None for value in state['limits'].values()))
        self.assertEqual(state['counters']['timeout'], 0)
        self.assertEqual(state['last_message']['vx'], .2)
        self.assertEqual(state['last_message']['source_age_at_receive_sec'], 0.)

    def test_configured_receive_timeout_is_not_a_source_clock_qualification(self):
        receiver = self.receiver(timeout_sec=.2)
        receiver.receive(message(), 10., 100001000000, 1)
        state = receiver.snapshot(10.1, 100101000000)
        self.assertEqual(state['receive_freshness'], 'WITHIN_RECEIVE_LIMIT')
        self.assertEqual(state['freshness'], 'UNCONFIGURED')

    def test_explicit_clock_limits_can_report_current_while_authority_stays_false(self):
        receiver = self.receiver(timeout_sec=.2, max_source_age_sec=.2, future_tolerance_sec=.01)
        receiver.receive(message(), 10., 100001000000, 1)
        state = receiver.snapshot(10.1, 100101000000)
        self.assertEqual(state['freshness'], 'CURRENT')
        self.assertFalse(state['control_authority'])
        self.assertTrue(state['last_valid_is_historical'])

    def test_duplicate_and_out_of_order_do_not_renew_receive_time(self):
        receiver = self.receiver(timeout_sec=.2)
        receiver.receive(message(10), 10., 100010000000, 1)
        duplicate = receiver.receive(message(10), 10.15, 100160000000, 1)
        older = receiver.receive(message(9), 10.16, 100170000000, 1)
        self.assertIn('DUPLICATE_SEQUENCE', duplicate['reasons'])
        self.assertIn('OUT_OF_ORDER_SEQUENCE', older['reasons'])
        state = receiver.snapshot(10.21, 100220000000)
        self.assertAlmostEqual(state['last_valid_age_sec'], .21)
        self.assertEqual(state['freshness'], 'EXPIRED')
        self.assertEqual(state['counters']['duplicate'], 1)
        self.assertEqual(state['counters']['out_of_order'], 1)
        self.assertEqual(state['counters']['timeout'], 1)
        receiver.snapshot(10.3, 100310000000)
        self.assertEqual(receiver.counters['timeout'], 1)

    def test_fresh_invalid_retracts_latest_observation_and_recovery_keeps_incident(self):
        receiver = self.receiver(timeout_sec=.2)
        receiver.receive(message(), 10., 100001000000, 1)
        event = receiver.receive(message(2, valid=False), 10.01, 100002000000, 1)
        state = receiver.snapshot(10.02, 100020000000)
        self.assertFalse(event['accepted'])
        self.assertEqual(state['freshness'], 'LATEST_MESSAGE_INVALID')
        self.assertFalse(state['last_message']['valid'])
        anomaly_id = state['last_anomaly']['event_id']
        receiver.receive(message(3), 10.03, 100031000000, 1)
        state = receiver.snapshot(10.04, 100040000000)
        self.assertEqual(state['latest_errors'], [])
        self.assertEqual(state['last_anomaly']['event_id'], anomaly_id)
        self.assertEqual(state['counters']['invalid'], 1)

    def test_source_and_reset_changes_are_independent_local_events_not_bound_to_state(self):
        receiver = self.receiver()
        receiver.receive(message(100, reset_counter=8), 10., 100100000000, 1)
        reset = receiver.receive(message(1, reset_counter=9, stamp=NS(sec=100, nanosec=101000000)),
                                 10.1, 100101000000, 1)
        self.assertTrue(reset['accepted'])
        self.assertEqual(reset['sample']['reset_counter'], 9)
        source = receiver.receive(message(1, reset_counter=9, source_id='new-session',
                                          stamp=NS(sec=100, nanosec=102000000)),
                                  10.2, 100102000000, 1)
        self.assertTrue(source['accepted'])
        events = receiver.drain_events()
        stream_events = [item for item in events if item['kind'] == 'stream_change']
        self.assertEqual([item['reasons'] for item in stream_events], [['RESET_CHANGED'], ['SOURCE_CHANGED']])
        self.assertEqual(receiver.counters['reset_change'], 1)
        self.assertEqual(receiver.counters['source_change'], 1)
        self.assertFalse(receiver.snapshot(10.2, 100102000000)['control_authority'])

    def test_restart_without_new_identity_is_visible_and_not_silently_accepted(self):
        receiver = self.receiver()
        receiver.receive(message(100), 10., 100100000000, 1)
        event = receiver.receive(message(1), 10.1, 100101000000, 1)
        self.assertIn('OUT_OF_ORDER_SEQUENCE', event['reasons'])
        self.assertAlmostEqual(receiver.snapshot(10.1, 100101000000)['last_valid_age_sec'], .1)

    def test_publisher_conflict_and_recovery_preserve_conflict_evidence(self):
        receiver = self.receiver()
        receiver.receive(message(), 10., 100001000000, 1)
        receiver.observe_publishers(2, 10.01, 100011000000)
        receiver.observe_publishers(1, 10.02, 100021000000)
        receiver.receive(message(2), 10.03, 100002000000, 1)
        state = receiver.snapshot(10.04, 100041000000)
        self.assertEqual(state['publisher_count'], 1)
        self.assertEqual(state['counters']['source_count'], 1)
        self.assertEqual(state['last_anomaly']['reasons'], ['PUBLISHER_COUNT_NOT_ONE'])

    def test_frame_version_sequence_source_and_nonfinite_payload_checks(self):
        cases = [(dict(frame_id=''), 'FRAME_ID_MUST_BE_EXPLICIT_BODY_FRD'),
                 (dict(frame_id='base_link_frd'), 'FRAME_ID_MUST_BE_EXPLICIT_BODY_FRD'),
                 (dict(schema_version=2), 'SCHEMA_VERSION_INVALID'),
                 (dict(sequence=0), 'SEQUENCE_INVALID'),
                 (dict(source_id=' '), 'SOURCE_ID_INVALID'),
                 (dict(vx=math.nan), 'VELOCITY_NONFINITE_OR_MISSING'),
                 (dict(vy=math.inf), 'VELOCITY_NONFINITE_OR_MISSING'),
                 (dict(stamp=NS(sec=100, nanosec=1000000000)), 'STAMP_INVALID'),
                 (dict(stamp=NS(sec=0, nanosec=0)), 'STAMP_INVALID')]
        for values, reason in cases:
            with self.subTest(values=values):
                receiver = self.receiver()
                event = receiver.receive(message(**values), 10., 100001000000, 1)
                self.assertIn(reason, event['reasons'])
                state = receiver.snapshot(10., 100001000000)
                encoded = json.dumps(state, allow_nan=False)
                self.assertNotIn('NaN', encoded)
                if 'vx' in values and math.isnan(values['vx']):
                    self.assertIsNone(state['last_message']['vx'])

    def test_explicit_vector_speed_and_clock_limits_are_checked(self):
        receiver = self.receiver(max_speed_mps=.5, max_source_age_sec=.1, future_tolerance_sec=.02)
        event = receiver.receive(message(vx=.4, vy=.4, vz=0.), 10., 100201000000, 1)
        self.assertIn('SPEED_LIMIT_EXCEEDED', event['reasons'])
        self.assertIn('SOURCE_AGE_LIMIT_EXCEEDED', event['reasons'])
        event = self.receiver(future_tolerance_sec=.02).receive(message(), 10., 99900000000, 1)
        self.assertIn('SOURCE_STAMP_IN_FUTURE', event['reasons'])

    def test_same_stamp_cannot_be_reissued_under_a_new_sequence(self):
        receiver = self.receiver()
        receiver.receive(message(), 10., 100001000000, 1)
        event = receiver.receive(message(2, stamp=NS(sec=100, nanosec=1000000)),
                                 10.1, 100101000000, 1)
        self.assertIn('STAMP_NOT_ADVANCING', event['reasons'])
        self.assertAlmostEqual(receiver.snapshot(10.1, 100101000000)['last_valid_age_sec'], .1)

    def test_event_memory_is_bounded_and_drops_are_counted(self):
        receiver = self.receiver()
        for index in range(400):
            receiver.receive(message(index + 1), 10. + index / 100., 100001000000 + index * 1000000, 1)
        self.assertEqual(len(receiver.drain_events()), 128)
        self.assertGreater(receiver.counters['events_dropped'], 0)

    def test_invalid_configuration_rejected_without_implicit_defaults(self):
        for value in (-1., math.nan, math.inf, True):
            with self.subTest(value=value), self.assertRaises(ValueError):
                diagnostic.DiagnosticConfig(timeout_sec=value)

    def qualified(self):
        return self.receiver(timeout_sec=.3, max_speed_mps=.5, max_source_age_sec=.2,
                             future_tolerance_sec=.01, expected_source_id='synthetic-session-1')

    def test_cache_is_separate_from_history_and_invalid_message_revokes_it(self):
        receiver = self.qualified()
        receiver.receive(message(), 10., 100001000000, 1)
        state = receiver.snapshot(10.01, 100011000000)
        self.assertTrue(state['cache_usable'])
        self.assertEqual(state['current_candidate']['sequence'], 1)
        receiver.receive(message(2, valid=False), 10.02, 100021000000, 1)
        state = receiver.snapshot(10.03, 100031000000)
        self.assertFalse(state['cache_usable'])
        self.assertIsNone(state['current_candidate'])
        self.assertEqual(state['historical_last_accepted_message']['sequence'], 1)
        receiver.receive(message(3), 10.04, 100041000000, 1)
        self.assertTrue(receiver.snapshot(10.05, 100051000000)['cache_usable'])

    def test_zero_or_multiple_publishers_revoke_cache_until_a_new_message_arrives(self):
        for count in (0, 2):
            with self.subTest(count=count):
                receiver = self.qualified()
                receiver.receive(message(), 10., 100001000000, 1)
                state = receiver.snapshot(10.01, 100011000000, count)
                self.assertFalse(state['cache_usable'])
                state = receiver.snapshot(10.02, 100021000000, 1)
                self.assertFalse(state['cache_usable'])
                self.assertEqual(state['historical_last_accepted_message']['sequence'], 1)
                receiver.receive(message(2), 10.03, 100031000000, 1)
                self.assertTrue(receiver.snapshot(10.04, 100041000000)['cache_usable'])

    def test_source_age_is_recomputed_and_revokes_cache_without_new_packets(self):
        receiver = self.qualified()
        receiver.receive(message(), 10., 100001000000, 1)
        self.assertTrue(receiver.snapshot(10.01, 100011000000)['cache_usable'])
        state = receiver.snapshot(10.21, 100211000000)
        self.assertFalse(state['cache_usable'])
        self.assertIn('SOURCE_AGE_LIMIT_EXCEEDED', state['cache_errors'])
        self.assertIsNone(state['current_candidate'])
        self.assertEqual(state['counters']['source_expired'], 1)
        receiver.receive(message(2, stamp=NS(sec=100, nanosec=220000000)),
                         10.22, 100221000000, 1)
        self.assertTrue(receiver.snapshot(10.23, 100231000000)['cache_usable'])

    def test_clock_regression_revokes_cache_and_fresh_ordered_input_can_recover(self):
        receiver = self.qualified()
        receiver.receive(message(), 10., 100001000000, 1)
        state = receiver.snapshot(9.9, 100002000000)
        self.assertFalse(state['cache_usable'])
        self.assertIn('RECEIVE_CLOCK_REGRESSION', state['last_anomaly']['reasons'])
        receiver.receive(message(2), 9.91, 100003000000, 1)
        self.assertTrue(receiver.snapshot(9.92, 100004000000)['cache_usable'])
        state = receiver.snapshot(9.93, 100002000000)
        self.assertFalse(state['cache_usable'])
        self.assertIn('ROS_CLOCK_REGRESSION', state['last_anomaly']['reasons'])
        receiver.receive(message(3), 9.94, 100004000000, 1)
        self.assertTrue(receiver.snapshot(9.95, 100005000000)['cache_usable'])

    def test_source_stamp_regression_is_explicit_and_cannot_renew_cache(self):
        receiver = self.qualified()
        receiver.receive(message(10), 10., 100010000000, 1)
        event = receiver.receive(message(11, stamp=NS(sec=100, nanosec=9000000)),
                                 10.01, 100020000000, 1)
        self.assertIn('SOURCE_CLOCK_REGRESSION', event['reasons'])
        state = receiver.snapshot(10.02, 100030000000)
        self.assertFalse(state['cache_usable'])
        self.assertEqual(state['counters']['clock_regression'], 1)
        self.assertEqual(state['historical_last_accepted_message']['sequence'], 10)

    def test_bound_source_mismatch_never_becomes_a_current_candidate(self):
        receiver = self.qualified()
        receiver.receive(message(), 10., 100001000000, 1)
        event = receiver.receive(message(2, source_id='unexpected-session'), 10.01, 100011000000, 1)
        self.assertIn('SOURCE_ID_NOT_EXPECTED', event['reasons'])
        state = receiver.snapshot(10.02, 100021000000)
        self.assertFalse(state['cache_usable'])
        self.assertEqual(state['last_message']['source_id'], 'unexpected-session')
        self.assertEqual(state['historical_last_accepted_message']['source_id'], 'synthetic-session-1')

    def test_unbound_source_or_unconfigured_speed_cannot_qualify_cache(self):
        receiver = self.receiver(timeout_sec=.2, max_source_age_sec=.2, future_tolerance_sec=.01,
                                 expected_source_id='synthetic-session-1')
        receiver.receive(message(), 10., 100001000000, 1)
        state = receiver.snapshot(10.01, 100011000000)
        self.assertFalse(state['cache_usable'])
        self.assertIn('LIMITS_UNCONFIGURED', state['cache_errors'])

    def test_a_to_b_to_a_cannot_renew_a_previously_observed_packet(self):
        receiver = self.qualified()
        receiver.receive(message(100), 10., 100100000000, 1)
        receiver.receive(message(1, source_id='unexpected-session'), 10.01, 100110000000, 1)
        event = receiver.receive(message(100), 10.02, 100120000000, 1)
        self.assertIn('DUPLICATE_SEQUENCE', event['reasons'])
        state = receiver.snapshot(10.03, 100130000000)
        self.assertFalse(state['cache_usable'])
        self.assertAlmostEqual(state['last_valid_age_sec'], .03)
        receiver.receive(message(101), 10.04, 100140000000, 1)
        self.assertTrue(receiver.snapshot(10.05, 100150000000)['cache_usable'])

    def test_rejected_future_or_malformed_packets_cannot_poison_trusted_watermarks(self):
        faults = [dict(stamp=NS(sec=10000, nanosec=0)), dict(schema_version=2),
                  dict(frame_id='unknown'), dict(vx=math.nan), dict(vx=1.e100)]
        for fault in faults:
            with self.subTest(fault=fault):
                receiver = self.qualified()
                receiver.receive(message(), 10., 100001000000, 1)
                values = dict(stamp=NS(sec=100, nanosec=2000000))
                values.update(fault)
                bad = receiver.receive(message(2**64 - 1, **values), 10.01, 100011000000, 1)
                self.assertFalse(bad['accepted'])
                good = receiver.receive(message(2), 10.02, 100021000000, 1)
                self.assertTrue(good['accepted'])
                state = receiver.snapshot(10.03, 100031000000)
                self.assertTrue(state['cache_usable'])
                self.assertGreater(state['counters']['invalid'], 0)
                self.assertEqual(state['last_anomaly']['event_id'], bad['event_id'])

    def test_explicit_withdrawal_cannot_be_reissued_with_same_sequence_as_valid(self):
        receiver = self.qualified()
        receiver.receive(message(), 10., 100001000000, 1)
        receiver.receive(message(2, valid=False), 10.01, 100011000000, 1)
        replay = receiver.receive(message(2), 10.02, 100021000000, 1)
        self.assertIn('DUPLICATE_SEQUENCE', replay['reasons'])
        receiver.receive(message(3), 10.03, 100031000000, 1)
        self.assertTrue(receiver.snapshot(10.04, 100041000000)['cache_usable'])

    def test_receive_gap_is_recorded_before_a_healthy_tail_replaces_its_age(self):
        receiver = self.qualified()
        receiver.receive(message(), 10., 100001000000, 1)
        event = receiver.receive(message(2, stamp=NS(sec=100, nanosec=401000000)),
                                 10.4, 100401000000, 1)
        self.assertTrue(event['accepted'])
        state = receiver.snapshot(10.41, 100411000000)
        self.assertTrue(state['cache_usable'])
        self.assertEqual(state['counters']['timeout'], 1)
        self.assertEqual(state['last_anomaly']['kind'], 'timeout')
        self.assertEqual(state['last_anomaly']['last_valid_sequence'], 1)
        self.assertAlmostEqual(state['last_anomaly']['last_valid_age_sec'], .4)

    def test_stream_history_capacity_never_evicts_old_replay_evidence(self):
        receiver = self.qualified()
        for reset in range(diagnostic.STREAM_HISTORY_CAPACITY):
            accepted = receiver.receive(message(reset_counter=reset,
                                               stamp=NS(sec=100, nanosec=(reset + 1) * 1000000)),
                                        10. + reset * .001,
                                        100001000000 + reset * 1000000, 1)
            self.assertTrue(accepted['accepted'])
        full = receiver.receive(message(reset_counter=diagnostic.STREAM_HISTORY_CAPACITY),
                                10.033, 100034000000, 1)
        self.assertIn('STREAM_HISTORY_CAPACITY_REACHED', full['reasons'])
        state = receiver.snapshot(10.034, 100035000000)
        self.assertEqual(state['stream_history_size'], diagnostic.STREAM_HISTORY_CAPACITY)
        self.assertFalse(state['cache_usable'])
        replay = receiver.receive(message(reset_counter=0), 10.035, 100036000000, 1)
        self.assertIn('DUPLICATE_SEQUENCE', replay['reasons'])
        self.assertFalse(receiver.snapshot(10.036, 100037000000)['cache_usable'])
        recovered = receiver.receive(message(2, reset_counter=31, stamp=NS(sec=100, nanosec=40000000)),
                                     10.04, 100041000000, 1)
        self.assertTrue(recovered['accepted'])
        state = receiver.snapshot(10.041, 100042000000)
        self.assertTrue(state['cache_usable'])
        self.assertEqual(state['current_candidate']['reset_counter'], 31)
        self.assertEqual(state['stream_history_size'], diagnostic.STREAM_HISTORY_CAPACITY)

    def test_rejected_identity_does_not_consume_stream_history_capacity(self):
        receiver = self.qualified()
        for reset in range(40):
            rejected = receiver.receive(message(reset_counter=reset, vx=math.nan),
                                        10. + reset * .001, 100001000000 + reset * 1000000, 1)
            self.assertFalse(rejected['accepted'])
        state = receiver.snapshot(10.04, 100041000000)
        self.assertEqual(state['stream_history_size'], 0)
        self.assertTrue(receiver.receive(message(reset_counter=40), 10.041, 100042000000, 1)['accepted'])
        self.assertEqual(receiver.snapshot(10.042, 100043000000)['stream_history_size'], 1)

    def test_reset_cannot_restart_source_clock_but_new_stamp_can_recover(self):
        for stamp in (NS(sec=100, nanosec=5000000), NS(sec=100, nanosec=10000000)):
            with self.subTest(stamp=stamp):
                receiver = self.qualified()
                receiver.receive(message(10), 10., 100010000000, 1)
                rejected = receiver.receive(message(1, reset_counter=1, stamp=stamp),
                                            10.01, 100020000000, 1)
                self.assertIn('SOURCE_STAMP_NOT_ADVANCING_ACROSS_RESET', rejected['reasons'])
                state = receiver.snapshot(10.02, 100030000000)
                self.assertFalse(state['cache_usable'])
                self.assertEqual(state['stream_history_size'], 1)
                recovered = receiver.receive(message(1, reset_counter=1,
                                                     stamp=NS(sec=100, nanosec=40000000)),
                                             10.03, 100040000000, 1)
                self.assertTrue(recovered['accepted'])
                self.assertTrue(receiver.snapshot(10.04, 100050000000)['cache_usable'])

    def test_valid_withdrawal_also_establishes_cross_reset_source_stamp(self):
        receiver = self.qualified()
        receiver.receive(message(), 10., 100001000000, 1)
        receiver.receive(message(2, valid=False, stamp=NS(sec=100, nanosec=2000000)),
                         10.01, 100011000000, 1)
        rejected = receiver.receive(message(1, reset_counter=1,
                                            stamp=NS(sec=100, nanosec=2000000)),
                                    10.02, 100021000000, 1)
        self.assertIn('SOURCE_STAMP_NOT_ADVANCING_ACROSS_RESET', rejected['reasons'])
        recovered = receiver.receive(message(1, reset_counter=1,
                                             stamp=NS(sec=100, nanosec=3000000)),
                                     10.03, 100031000000, 1)
        self.assertTrue(recovered['accepted'])


class WriterTests(unittest.TestCase):
    def test_default_writer_creates_nothing(self):
        writer = diagnostic.DiagnosticWriter()
        self.assertFalse(writer.record(dict(kind='message'), 1.))
        writer.snapshot(dict(control_authority=False))
        self.assertIsNone(writer.directory)

    def test_log_rotation_and_snapshot_are_bounded_json_evidence(self):
        with tempfile.TemporaryDirectory(prefix='fc01_velocity_diag_writer_') as directory:
            writer = diagnostic.DiagnosticWriter(directory, log_max_bytes=1024,
                                                  log_backup_count=1, log_min_interval_sec=0.)
            for number in range(100):
                writer.record(dict(kind='message', event_id=number, reasons=[], payload='x' * 200), float(number))
            writer.snapshot(dict(control_authority=False, missing=math.nan))
            root = Path(directory)
            self.assertEqual({path.name for path in root.iterdir()},
                             {'velocity_command_diagnostic.json', 'velocity_command_diagnostic.log',
                              'velocity_command_diagnostic.log.1'})
            for path in root.glob('*.log*'):
                self.assertLessEqual(path.stat().st_size, 1024)
                for line in path.read_text().splitlines():
                    self.assertEqual(json.loads(line)['kind'], 'message')
            snapshot = json.loads((root / 'velocity_command_diagnostic.json').read_text())
            self.assertIsNone(snapshot['missing'])
            self.assertFalse(snapshot['control_authority'])

    def test_throttled_unique_anomaly_survives_following_healthy_data(self):
        with tempfile.TemporaryDirectory(prefix='fc01_velocity_diag_throttle_') as directory:
            writer = diagnostic.DiagnosticWriter(directory, log_min_interval_sec=1.)
            writer.record(dict(kind='message', event_id=1, reasons=[]), 10.)
            writer.record(dict(kind='message', event_id=2, reasons=['COMMAND_INVALID']), 10.1)
            writer.record(dict(kind='message', event_id=3, reasons=[]), 10.2)
            self.assertFalse(writer.flush(10.3))
            self.assertTrue(writer.flush(11.1))
            lines = [json.loads(line) for line in (Path(directory) / 'velocity_command_diagnostic.log').read_text().splitlines()]
            self.assertEqual(len(lines), 2)
            self.assertEqual(lines[-1]['reasons'], ['COMMAND_INVALID'])
            self.assertEqual(lines[-1]['log_events_suppressed'], 2)

    def test_oversize_event_cannot_break_file_limit(self):
        with tempfile.TemporaryDirectory(prefix='fc01_velocity_diag_oversize_') as directory:
            writer = diagnostic.DiagnosticWriter(directory, log_max_bytes=1024)
            writer.record(dict(kind='message', event_id=1, payload='z' * 10000), 10.)
            path = Path(directory) / 'velocity_command_diagnostic.log'
            self.assertLessEqual(path.stat().st_size, 1024)
            self.assertEqual(json.loads(path.read_text())['reasons'], ['LOG_RECORD_TOO_LARGE'])

    def test_source_contains_no_publisher_or_flight_module_and_ros_imports_only_in_main(self):
        tree = ast.parse(SCRIPT.read_text())
        self.assertNotIn('create_publisher', SCRIPT.read_text())
        self.assertNotIn('NavigationCommand', SCRIPT.read_text())
        self.assertFalse(any(isinstance(item, ast.FunctionDef) and item.name == 'context'
                             for item in ast.walk(tree)))
        for item in tree.body:
            if isinstance(item, (ast.Import, ast.ImportFrom)):
                names = [alias.name for alias in item.names]
                self.assertFalse(any(name.startswith(('rclpy', 'jikl_fc01_system')) for name in names))
                self.assertFalse((getattr(item, 'module', '') or '').startswith(('rclpy', 'jikl_fc01_system')))


if __name__ == '__main__':
    unittest.main()
