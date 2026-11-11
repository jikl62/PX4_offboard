"""Pure state validation with real conversion and synthetic Odometry/Health."""
from copy import deepcopy
import importlib.util
import math
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    'fc01_test_vision_state_validation', ROOT/'native/tools/vision_state_validation.py')
m = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = m
SPEC.loader.exec_module(m)


def stamp(value):
    sec, nanosec = divmod(value, 1000000000)
    return NS(sec=sec, nanosec=nanosec)


def vector(values):
    return NS(**dict(zip(('x', 'y', 'z'), values)))


def diagonal(values=(.01,)*6):
    return [values[i] if i == j else 0. for i in range(6) for j in range(6)]


def adapter_config(**changes):
    values = dict(source_id='synthetic-session', input_frame_id='raw_world',
                  input_body_frame_id='raw_body', world_convention='NED', body_convention='FRD',
                  body_from_sensor_wxyz=(1., 0., 0., 0.), sensor_position_body_frd=(0., 0., 0.),
                  world_alignment_wxyz=(1., 0., 0., 0.), world_translation_ned=(0., 0., 0.),
                  extrinsics_id='synthetic-mount', alignment_id='synthetic-alignment')
    return m.AdapterConfig(**dict(values, **changes))


def config(**changes):
    values = dict(adapter=adapter_config(), clock=m.ClockPolicy(offset_ns=0, confirmed=True),
                  receive_timeout_sec=.25, max_source_age_sec=.30,
                  future_tolerance_sec=.01, min_quality=.4, nominal_quality_min=.8)
    return m.ValidationConfig(**dict(values, **changes))


def pair(sequence=1, stamp_ns=None, position=(0., 0., 0.), velocity=(0., 0., 0.),
         attitude=(1., 0., 0., 0.), omega=(0., 0., 0.), **health_changes):
    measurement_stamp = 10000000000+sequence*50000000 if stamp_ns is None else stamp_ns
    raw = NS(header=NS(stamp=stamp(measurement_stamp), frame_id='raw_world'),
             child_frame_id='raw_body',
             pose=NS(pose=NS(position=vector(position),
                             orientation=NS(**dict(zip(('w', 'x', 'y', 'z'), attitude)))),
                     covariance=diagonal()),
             twist=NS(twist=NS(linear=vector(velocity), angular=vector(omega)),
                      covariance=diagonal()))
    values = dict(schema_version=1, sequence=sequence, stamp=stamp(measurement_stamp),
                  source_id='synthetic-session', frame_id='raw_world', body_frame_id='raw_body',
                  reset_counter=0, angular_velocity_source='synthetic-gyro', quality=.9,
                  extrinsics_id='synthetic-mount', alignment_id='synthetic-alignment')
    for name in ('reset_valid', 'initialized', 'tracking', 'calibration_valid',
                 'frame_alignment_valid', 'time_sync_valid', 'extrinsics_valid',
                 'metric_scale_valid', 'quality_valid', 'position_valid',
                 'velocity_valid', 'attitude_valid', 'angular_velocity_valid'):
        values[name] = True
    return raw, NS(**dict(values, **health_changes))


def feed(validator, sequence=1, *, now=None, ros_now=None, metadata=None, **changes):
    raw, health = pair(sequence, **changes)
    received = 1.+sequence*.05 if now is None else now
    source_stamp = raw.header.stamp.sec*1000000000+raw.header.stamp.nanosec
    return validator.receive(raw, health, received,
                             source_stamp+10000000 if ros_now is None else ros_now, metadata)


class StateValidationTests(unittest.TestCase):
    def test_normal_stationary_and_moving_samples_keep_state_and_target_separate(self):
        validator = m.Validator(config())
        first = feed(validator)
        self.assertTrue(first.accepted)
        self.assertEqual(first.state.position, (0., 0., 0.))
        moving = feed(validator, 2, position=(.01, -.02, -.03), velocity=(.1, -.2, -.3))
        self.assertTrue(moving.accepted)
        self.assertEqual(moving.state.velocity, (.1, -.2, -.3))
        self.assertEqual(moving.diagnostics['tracking_status'], 'NORMAL')
        self.assertFalse(moving.control_authority)
        self.assertEqual(moving.diagnostics['qualification'], 'DIAGNOSTIC_ONLY')
        self.assertFalse(hasattr(moving, 'target_velocity'))

    def test_enu_flu_conversion_uses_actual_converter(self):
        validator = m.Validator(config(adapter=adapter_config(world_convention='ENU', body_convention='FLU')))
        result = feed(validator, position=(1., 2., 3.), velocity=(.4, .5, .6))
        self.assertTrue(result.accepted)
        self.assertEqual(result.state.position, (2., 1., -3.))
        self.assertEqual(result.state.velocity, (.5, .4, -.6))
        self.assertAlmostEqual(sum(x*x for x in result.state.attitude), 1.)

    def test_lever_arm_uses_actual_angular_velocity_correction(self):
        validator = m.Validator(config(adapter=adapter_config(sensor_position_body_frd=(1., 0., 0.))))
        result = feed(validator, position=(2., 0., 0.), velocity=(0., 1., 0.), omega=(0., 0., 1.))
        self.assertTrue(result.accepted)
        self.assertEqual(result.state.position, (1., 0., 0.))
        self.assertEqual(result.state.velocity, (0., 0., 0.))

    def test_explicit_clock_offset_applies_to_copies_only(self):
        validator = m.Validator(config(clock=m.ClockPolicy(offset_ns=5000000000, confirmed=True)))
        raw, health = pair(stamp_ns=5000000000)
        before_raw, before_health = deepcopy(raw), deepcopy(health)
        result = validator.receive(raw, health, 1., 10010000000)
        self.assertTrue(result.accepted)
        self.assertEqual(result.state.stamp_ns, 10000000000)
        self.assertEqual(raw, before_raw)
        self.assertEqual(health, before_health)
        self.assertEqual(result.diagnostics['raw_stamp_ns'], 5000000000)

    def test_unconfirmed_clock_and_missing_policy_remain_unknown(self):
        validator = m.Validator(m.ValidationConfig(adapter_config(), clock=m.ClockPolicy(offset_ns=5000000000)))
        result = feed(validator)
        self.assertFalse(result.accepted)
        self.assertEqual(result.state.valid_fields, 0)
        self.assertIn('clock_offset', result.diagnostics['unknown'])
        self.assertIn('min_quality', result.diagnostics['unknown'])
        self.assertIn('CLOCK_OFFSET_UNCONFIRMED', result.diagnostics['reasons'])
        self.assertIn('TIMING_POLICY_UNCONFIGURED', result.diagnostics['reasons'])

    def test_stale_and_future_source_times_do_not_poison_next_good_sample(self):
        validator = m.Validator(config())
        self.assertTrue(feed(validator).accepted)
        stale = feed(validator, 100, ros_now=16000000000)
        self.assertIn('SOURCE_SAMPLE_EXPIRED', stale.diagnostics['reasons'])
        recovered = feed(validator, 2, now=6.05, ros_now=16010000000, stamp_ns=16000000000)
        self.assertTrue(recovered.accepted)
        future = feed(validator, 999, now=6.1, stamp_ns=20000000000, ros_now=16100000000)
        self.assertIn('SOURCE_SAMPLE_IN_FUTURE', future.diagnostics['reasons'])
        recovered = feed(validator, 3, now=6.2, stamp_ns=16200000000, ros_now=16210000000)
        self.assertTrue(recovered.accepted)

    def test_current_expiry_revokes_cache_and_new_data_recovers(self):
        validator = m.Validator(config(max_source_age_sec=1.))
        self.assertTrue(feed(validator).accepted)
        self.assertTrue(validator.current(1.20, 10210000000).accepted)
        expired = validator.current(1.30, 10310000000)
        self.assertFalse(expired.accepted)
        self.assertEqual(expired.state.valid_fields, 0)
        self.assertIn('RECEIVE_SAMPLE_EXPIRED', expired.diagnostics['reasons'])
        again = validator.current(1.31, 10320000000)
        self.assertEqual(again.diagnostics['counters']['expired'], 1)
        recovered = feed(validator, 2, now=1.35, stamp_ns=10350000000)
        self.assertTrue(recovered.accepted)
        self.assertEqual(recovered.diagnostics['last_anomaly']['kind'], 'expired')

    def test_new_receive_records_prior_expiry_without_relying_on_timer(self):
        validator = m.Validator(config())
        self.assertTrue(feed(validator).accepted)
        recovered = feed(validator, 2, now=1.5, stamp_ns=10500000000)
        self.assertTrue(recovered.accepted)
        self.assertEqual(recovered.diagnostics['counters']['expired'], 1)
        self.assertEqual(recovered.diagnostics['counters']['recovered'], 1)
        anomaly = recovered.diagnostics['last_anomaly']
        self.assertTrue(anomaly['observed_on_receive'])
        self.assertIn('RECEIVE_SAMPLE_EXPIRED', anomaly['reasons'])
        self.assertTrue(feed(validator, 3, now=1.55, stamp_ns=10550000000).accepted)
        self.assertEqual(validator.last_anomaly, anomaly)

    def test_source_age_expires_even_with_recent_receive_and_current_unknown_clock(self):
        validator = m.Validator(config(receive_timeout_sec=2., max_source_age_sec=.1))
        feed(validator)
        result = validator.current(1.10, 10150000000)
        self.assertFalse(result.accepted)
        self.assertIn('SOURCE_SAMPLE_OUTSIDE_TIME_WINDOW', result.diagnostics['reasons'])
        self.assertTrue(feed(validator, 2, now=1.2, stamp_ns=10200000000).accepted)
        self.assertFalse(validator.current(1.21, 0).accepted)

    def test_expired_cache_is_recorded_before_retired_epoch_rejection_and_recovery(self):
        validator = m.Validator(config(max_position_step_m=.1))
        self.assertTrue(feed(validator, 1, now=1., stamp_ns=10000000000).accepted)
        self.assertTrue(feed(validator, 1, now=1.05, stamp_ns=10050000000,
                             reset_counter=1, position=(10., 0., 0.)).accepted)
        validator.drain_events()
        rejected = feed(validator, 99, now=1.5, stamp_ns=10500000000,
                        reset_counter=0, position=(0., 0., 0.))
        self.assertFalse(rejected.accepted)
        self.assertIn('RETIRED_EPOCH_REPLAY', rejected.diagnostics['reasons'])
        self.assertEqual(rejected.diagnostics['counters'].get('expired', 0), 1)
        self.assertEqual(rejected.diagnostics['trusted_epoch_count'], 2)
        events = validator.drain_events()
        expiry = next(event for event in events if event['kind'] == 'expired')
        refusal = next(event for event in events if event['kind'] == 'rejected')
        self.assertLess(expiry['event_id'], refusal['event_id'])
        self.assertIn('RECEIVE_SAMPLE_EXPIRED', expiry['reasons'])
        self.assertTrue(expiry['observed_on_receive'])
        recovered = feed(validator, 2, now=1.55, stamp_ns=10550000000,
                         reset_counter=1, position=(10., 0., 0.))
        self.assertTrue(recovered.accepted)
        self.assertEqual(recovered.diagnostics['counters']['expired'], 1)
        self.assertEqual(recovered.diagnostics['last_anomaly'], refusal)

    def test_invalid_identity_cannot_erase_expiry_or_poison_trusted_state(self):
        for source, reason in (('unknown-source', 'SOURCE_FRAME_OR_CALIBRATION_ID_MISMATCH'),
                               ('', 'EPOCH_IDENTITY_INVALID')):
            with self.subTest(source=source):
                validator = m.Validator(config(max_position_step_m=.1))
                self.assertTrue(feed(validator, 1, now=1., stamp_ns=10000000000).accepted)
                validator.drain_events()
                rejected = feed(validator, 999, now=1.4, stamp_ns=10400000000,
                                source_id=source, position=(100., 0., 0.))
                self.assertFalse(rejected.accepted)
                self.assertIn(reason, rejected.diagnostics['reasons'])
                self.assertEqual(rejected.diagnostics['counters'].get('expired', 0), 1)
                self.assertEqual(rejected.diagnostics['trusted_epoch_count'], 1)
                # Further rejection/current observations cannot count the same
                # already-revoked cache as another expired sample.
                self.assertFalse(feed(validator, 1000, now=1.42, stamp_ns=10420000000,
                                      source_id=source).accepted)
                self.assertFalse(validator.current(1.43, 10440000000).accepted)
                recovered = feed(validator, 2, now=1.45, stamp_ns=10450000000)
                self.assertTrue(recovered.accepted)
                self.assertEqual(recovered.diagnostics['counters']['expired'], 1)
                self.assertEqual(recovered.diagnostics['trusted_epoch_count'], 1)
                expiries = [event for event in validator.drain_events() if event['kind'] == 'expired']
                self.assertEqual(len(expiries), 1)

    def test_invalid_epoch_cannot_erase_source_age_expiry_with_recent_receipt(self):
        validator = m.Validator(config(receive_timeout_sec=1., max_source_age_sec=.05))
        self.assertTrue(feed(validator, 1, now=1., stamp_ns=10000000000).accepted)
        validator.drain_events()
        rejected = feed(validator, 2, now=1.1, stamp_ns=10100000000, source_id='')
        self.assertFalse(rejected.accepted)
        events = validator.drain_events()
        expiries = [event for event in events if event['kind'] == 'expired']
        self.assertEqual(len(expiries), 1)
        expiry = expiries[0]
        self.assertEqual(expiry['reasons'], ['SOURCE_SAMPLE_OUTSIDE_TIME_WINDOW'])
        self.assertTrue(feed(validator, 2, now=1.15, stamp_ns=10150000000).accepted)
        self.assertEqual(validator.counters['expired'], 1)

    def test_regressed_clock_does_not_create_an_expiry_before_epoch_rejection(self):
        for received, ros_now, reason in ((1.5, 9990000000, 'ROS_CLOCK_REGRESSION'),
                                          (.9, 10500000000, 'RECEIVE_CLOCK_REGRESSION')):
            with self.subTest(reason=reason):
                validator = m.Validator(config())
                self.assertTrue(feed(validator, 1, now=1., stamp_ns=10000000000).accepted)
                validator.drain_events()
                rejected = feed(validator, 99, now=received, ros_now=ros_now,
                                stamp_ns=10500000000, source_id='')
                self.assertFalse(rejected.accepted)
                self.assertIn(reason, rejected.diagnostics['reasons'])
                self.assertEqual(validator.counters['expired'], 0)
                self.assertFalse(any(event['kind'] == 'expired' for event in validator.drain_events()))
                self.assertTrue(feed(validator, 2, now=1.55, stamp_ns=10550000000).accepted)

    def test_duplicate_and_regression_revoke_without_renewing_age(self):
        validator = m.Validator(config())
        feed(validator, 4)
        duplicate = feed(validator, 4, now=1.25)
        self.assertIn('DUPLICATE_SEQUENCE', duplicate.diagnostics['reasons'])
        self.assertFalse(validator.current(1.26, 10210000000).accepted)
        regression = feed(validator, 3, now=1.30)
        self.assertIn('SEQUENCE_REGRESSION', regression.diagnostics['reasons'])
        self.assertTrue(feed(validator, 5, now=1.35).accepted)

    def test_measurement_clock_regression_and_new_epoch_reset(self):
        validator = m.Validator(config())
        feed(validator, 4)
        result = feed(validator, 5, now=1.25, stamp_ns=10100000000, ros_now=10210000000)
        self.assertIn('MEASUREMENT_CLOCK_NOT_ADVANCING', result.diagnostics['reasons'])
        recovered = feed(validator, 1, now=1.30, stamp_ns=10220000000,
                         ros_now=10230000000, reset_counter=1)
        self.assertTrue(recovered.accepted)
        self.assertIn('ESTIMATOR_RESET', [reason for e in validator.drain_events() for reason in e['reasons']])

    def test_receive_and_ros_clock_regressions_are_visible_and_recoverable(self):
        validator = m.Validator(config())
        feed(validator)
        result = feed(validator, 2, now=.9, ros_now=10060000000)
        self.assertIn('RECEIVE_CLOCK_REGRESSION', result.diagnostics['reasons'])
        feed(validator, 3, now=1.2, ros_now=10160000000)
        result = feed(validator, 4, now=1.3, ros_now=10150000000)
        self.assertIn('ROS_CLOCK_REGRESSION', result.diagnostics['reasons'])
        self.assertTrue(feed(validator, 5, now=1.4).accepted)

    def test_tracking_degraded_lost_and_repeated_recovery_keep_anomaly(self):
        validator = m.Validator(config())
        self.assertEqual(feed(validator).diagnostics['tracking_status'], 'NORMAL')
        degraded = feed(validator, 2, quality=.6)
        self.assertTrue(degraded.accepted)
        self.assertEqual(degraded.diagnostics['tracking_status'], 'DEGRADED')
        for sequence in (3, 5, 7):
            lost = feed(validator, sequence, tracking=False)
            self.assertFalse(lost.accepted)
            self.assertEqual(lost.diagnostics['tracking_status'], 'LOST')
            recovered = feed(validator, sequence+1)
            self.assertTrue(recovered.accepted)
            self.assertEqual(recovered.diagnostics['last_anomaly']['kind'], 'rejected')
        self.assertEqual(recovered.diagnostics['counters']['recovered'], 3)

    def test_below_quality_threshold_rejects_and_unknown_threshold_stays_explicit(self):
        validator = m.Validator(config())
        result = feed(validator, quality=.1)
        self.assertIn('QUALITY_BELOW_EXPLICIT_MINIMUM', result.diagnostics['reasons'])
        validator = m.Validator(config(min_quality=None, nominal_quality_min=None))
        result = feed(validator, quality=.1)
        self.assertTrue(result.accepted)
        self.assertEqual(result.diagnostics['tracking_status'], 'TRACKING_UNCLASSIFIED')
        self.assertIn('min_quality', result.diagnostics['unknown'])

    def test_explicit_relocalization_and_reset_rebaseline_position_velocity_attitude(self):
        validator = m.Validator(config(max_position_step_m=.1, max_velocity_step_mps=.1,
                                       max_attitude_step_rad=.1))
        feed(validator)
        relocated = feed(validator, 2, position=(10., 2., 1.), velocity=(1., 2., 3.),
                         attitude=(0., 0., 0., 1.), metadata=m.SampleMetadata(relocalization_id='map-2'))
        self.assertTrue(relocated.accepted)
        reset = feed(validator, 1, now=1.2, stamp_ns=10200000000,
                     position=(-10., 0., 0.), reset_counter=1)
        self.assertTrue(reset.accepted)
        reasons = [reason for e in validator.drain_events() for reason in e['reasons']]
        self.assertIn('RELOCALIZATION', reasons)
        self.assertIn('ESTIMATOR_RESET', reasons)

    def test_source_and_frame_epoch_events_never_authorize_unconfigured_transforms(self):
        validator = m.Validator(config())
        feed(validator)
        unknown_source = feed(validator, 2, source_id='unknown-source')
        self.assertFalse(unknown_source.accepted)
        raw, health = pair(3, frame_id='new_map')
        raw.header.frame_id = 'new_map'
        result = validator.receive(raw, health, 1.2, 10160000000)
        self.assertFalse(result.accepted)
        reasons = [reason for e in validator.drain_events() for reason in e['reasons']]
        self.assertIn('SOURCE_EPOCH_CHANGED', reasons)
        self.assertIn('FRAME_EPOCH_CHANGED', reasons)
        self.assertTrue(feed(validator, 4, now=1.3).accepted)

    def test_rejected_unknown_epoch_cannot_reauthorize_duplicate_old_pair(self):
        validator = m.Validator(config())
        feed(validator, 3)
        feed(validator, 99, now=1.2, source_id='wrong-source')
        replay = feed(validator, 3, now=1.25)
        self.assertFalse(replay.accepted)
        self.assertIn('DUPLICATE_SEQUENCE', replay.diagnostics['reasons'])

    def test_retired_reset_epoch_replay_cannot_rebase_to_old_pose_within_ttl(self):
        validator = m.Validator(config(max_position_step_m=.1))
        self.assertTrue(feed(validator, 1, now=1., stamp_ns=10000000000).accepted)
        self.assertTrue(feed(validator, 1, now=1.05, stamp_ns=10050000000,
                             reset_counter=1, position=(10., 0., 0.)).accepted)
        replay = feed(validator, 1, now=1.1, stamp_ns=10000000000, ros_now=10110000000)
        self.assertFalse(replay.accepted)
        self.assertIn('RETIRED_EPOCH_REPLAY', replay.diagnostics['reasons'])
        self.assertEqual(replay.state.valid_fields, 0)
        self.assertFalse(validator.current(1.11, 10120000000).accepted)
        recovered = feed(validator, 2, now=1.15, stamp_ns=10150000000,
                         reset_counter=1, position=(10., 0., 0.))
        self.assertTrue(recovered.accepted)
        self.assertEqual(recovered.diagnostics['trusted_epoch_count'], 2)
        self.assertIn('RETIRED_EPOCH_REPLAY', recovered.diagnostics['last_anomaly']['reasons'])

    def test_new_epoch_must_advance_same_confirmed_measurement_clock(self):
        validator = m.Validator(config(max_position_step_m=.1))
        self.assertTrue(feed(validator, 5, now=1., stamp_ns=10050000000).accepted)
        for received, raw_stamp, ros_now in (
            (1.05, 10050000000, 10070000000),
            (1.10, 10040000000, 10080000000)):
            rejected = feed(validator, 1, now=received, stamp_ns=raw_stamp, ros_now=ros_now,
                            reset_counter=1, position=(10., 0., 0.))
            self.assertFalse(rejected.accepted)
            self.assertIn('MEASUREMENT_CLOCK_NOT_ADVANCING', rejected.diagnostics['reasons'])
            self.assertEqual(rejected.diagnostics['trusted_epoch_count'], 1)
        # The rejected reset did not retire the current epoch or move clocks.
        self.assertTrue(feed(validator, 6, now=1.15, stamp_ns=10090000000).accepted)
        advanced = feed(validator, 1, now=1.20, stamp_ns=10150000000,
                        reset_counter=1, position=(10., 0., 0.))
        self.assertTrue(advanced.accepted)
        self.assertEqual(advanced.diagnostics['trusted_epoch_count'], 2)

    def test_new_relocalization_epoch_cannot_repeat_confirmed_clock_stamp(self):
        validator = m.Validator(config())
        self.assertTrue(feed(validator, 1, now=1., stamp_ns=10000000000,
                             metadata=m.SampleMetadata(relocalization_id='map-A')).accepted)
        result = feed(validator, 1, now=1.05, stamp_ns=10000000000, ros_now=10060000000,
                      metadata=m.SampleMetadata(relocalization_id='map-B'))
        self.assertFalse(result.accepted)
        self.assertIn('MEASUREMENT_CLOCK_NOT_ADVANCING', result.diagnostics['reasons'])
        self.assertTrue(feed(validator, 1, now=1.1, stamp_ns=10100000000,
                             metadata=m.SampleMetadata(relocalization_id='map-B')).accepted)

    def test_retired_relocalization_epoch_with_new_sequence_is_still_rejected(self):
        validator = m.Validator(config(max_position_step_m=.1))
        feed(validator, 1, metadata=m.SampleMetadata(relocalization_id='map-A'))
        result = feed(validator, 2, position=(10., 0., 0.),
                      metadata=m.SampleMetadata(relocalization_id='map-B'))
        self.assertTrue(result.accepted)
        replay = feed(validator, 3, position=(0., 0., 0.),
                      metadata=m.SampleMetadata(relocalization_id='map-A'))
        self.assertFalse(replay.accepted)
        self.assertIn('RETIRED_EPOCH_REPLAY', replay.diagnostics['reasons'])

    def test_untrusted_epoch_neither_consumes_capacity_nor_retires_good_epoch(self):
        validator = m.Validator(config(epoch_history_capacity=2, max_position_step_m=.1))
        self.assertTrue(feed(validator).accepted)
        invalid = feed(validator, 1, now=1.1, stamp_ns=10100000000, reset_counter=1,
                       position=(10., 0., 0.), tracking=False)
        self.assertFalse(invalid.accepted)
        self.assertEqual(invalid.diagnostics['trusted_epoch_count'], 1)
        self.assertTrue(feed(validator, 2, now=1.15, stamp_ns=10150000000).accepted)
        result = feed(validator, 1, now=1.2, stamp_ns=10200000000, reset_counter=1,
                      position=(10., 0., 0.))
        self.assertTrue(result.accepted)
        self.assertEqual(result.diagnostics['trusted_epoch_count'], 2)

    def test_many_explicit_new_epochs_work_until_capacity_without_silent_eviction(self):
        validator = m.Validator(config(epoch_history_capacity=3, max_position_step_m=.1))
        for reset in range(3):
            result = feed(validator, 1, now=1.+reset*.05,
                          stamp_ns=10000000000+reset*50000000,
                          reset_counter=reset, position=(reset*10., 0., 0.))
            self.assertTrue(result.accepted)
        full = feed(validator, 1, now=1.15, stamp_ns=10150000000,
                    reset_counter=3, position=(30., 0., 0.))
        self.assertFalse(full.accepted)
        self.assertIn('EPOCH_HISTORY_CAPACITY_REACHED', full.diagnostics['reasons'])
        self.assertEqual(full.diagnostics['trusted_epoch_count'], 3)
        replay = feed(validator, 2, now=1.2, stamp_ns=10200000000,
                      reset_counter=0, position=(0., 0., 0.))
        self.assertFalse(replay.accepted)
        self.assertIn('RETIRED_EPOCH_REPLAY', replay.diagnostics['reasons'])
        still_current_epoch = feed(validator, 2, now=1.25, stamp_ns=10250000000,
                                   reset_counter=2, position=(20., 0., 0.))
        self.assertTrue(still_current_epoch.accepted)
        fresh_offline_validator = m.Validator(config(epoch_history_capacity=3))
        self.assertTrue(feed(fresh_offline_validator, 1, now=1.3, stamp_ns=10300000000,
                             reset_counter=3).accepted)

    def test_unexplained_jump_cannot_rebaseline_itself_and_return_to_trust_recovers(self):
        for field, changes in (
            ('POSITION', dict(position=(2., 0., 0.))),
            ('VELOCITY', dict(velocity=(2., 0., 0.))),
            ('ATTITUDE', dict(attitude=(0., 0., 0., 1.)))):
            with self.subTest(field=field):
                validator = m.Validator(config(max_position_step_m=.1, max_velocity_step_mps=.1,
                                               max_attitude_step_rad=.1))
                feed(validator)
                result = feed(validator, 2, **changes)
                self.assertIn(field+'_CONTINUITY_LIMIT_EXCEEDED', result.diagnostics['reasons'])
                repeated = feed(validator, 3, **changes)
                self.assertFalse(repeated.accepted)
                self.assertIn(field+'_CONTINUITY_LIMIT_EXCEEDED', repeated.diagnostics['reasons'])
                self.assertTrue(feed(validator, 4).accepted)
                self.assertIsNotNone(validator.last_anomaly)

    def test_expiry_does_not_erase_last_trusted_continuity_baseline(self):
        validator = m.Validator(config(max_position_step_m=.1))
        feed(validator)
        self.assertFalse(validator.current(1.5, 10500000000).accepted)
        result = feed(validator, 2, now=1.6, stamp_ns=10600000000, position=(2., 0., 0.))
        self.assertFalse(result.accepted)
        self.assertIn('POSITION_CONTINUITY_LIMIT_EXCEEDED', result.diagnostics['reasons'])
        recovered = feed(validator, 3, now=1.7, stamp_ns=10700000000,
                         position=(2., 0., 0.), reset_counter=1)
        self.assertTrue(recovered.accepted)

    def test_snapshot_clock_watermark_covers_receive_and_empty_cache(self):
        validator = m.Validator(config())
        feed(validator)
        self.assertTrue(validator.current(1.20, 10200000000).accepted)
        regression = validator.current(1.15, 10210000000)
        self.assertFalse(regression.accepted)
        self.assertIn('RECEIVE_CLOCK_REGRESSION', regression.diagnostics['reasons'])
        self.assertFalse(feed(validator, 2, now=1.19, stamp_ns=10220000000).accepted)
        self.assertTrue(feed(validator, 3, now=1.21, stamp_ns=10230000000).accepted)
        validator = m.Validator(config())
        validator.current(10., 20000000000)
        regression = validator.current(9., 19000000000)
        self.assertIn('RECEIVE_CLOCK_REGRESSION', regression.diagnostics['reasons'])
        self.assertIn('ROS_CLOCK_REGRESSION', regression.diagnostics['reasons'])
        self.assertFalse(feed(validator, now=9.5, stamp_ns=19500000000).accepted)

    def test_no_step_limits_does_not_invent_jump_faults_and_q_sign_is_equivalent(self):
        validator = m.Validator(config())
        feed(validator)
        self.assertTrue(feed(validator, 2, position=(100., 0., 0.)).accepted)
        validator = m.Validator(config(max_attitude_step_rad=.01))
        feed(validator)
        self.assertTrue(feed(validator, 2, attitude=(-1., 0., 0., 0.)).accepted)

    def test_identical_attitude_and_its_opposite_sign_pass_tiny_step_limit(self):
        attitude = (
            -.5137551155216827, .5646433853044953,
            .5302328846745061, -.36889919587460945,
        )
        for sign in (1., -1.):
            with self.subTest(sign=sign):
                validator = m.Validator(config(max_attitude_step_rad=1.e-12))
                self.assertTrue(feed(validator, attitude=attitude).accepted)
                equivalent = tuple(sign*value for value in attitude)
                second = feed(validator, 2, attitude=equivalent)
                self.assertTrue(second.accepted, second.diagnostics['reasons'])

    def test_tiny_real_attitude_steps_obey_explicit_limit(self):
        for angle, expected in ((.5e-9, True), (2.e-9, False)):
            with self.subTest(angle=angle):
                validator = m.Validator(config(max_attitude_step_rad=1.e-9))
                self.assertTrue(feed(validator).accepted)
                attitude = (math.cos(angle/2), 0., 0., math.sin(angle/2))
                changed = feed(validator, 2, attitude=attitude)
                self.assertEqual(changed.accepted, expected)
                if not expected:
                    self.assertIn('ATTITUDE_CONTINUITY_LIMIT_EXCEEDED',
                                  changed.diagnostics['reasons'])

    def test_covariance_unknown_and_declared_zero_never_become_transformed_valid(self):
        validator = m.Validator(config())
        raw, health = pair()
        raw.pose.covariance = raw.twist.covariance = [0.]*36
        unknown = validator.receive(raw, health, 1., 10060000000)
        self.assertEqual(unknown.diagnostics['covariance']['pose']['status'], 'UNKNOWN')
        raw, health = pair(2)
        raw.pose.covariance = raw.twist.covariance = [0.]*36
        known = validator.receive(raw, health, 1.1, 10110000000,
                                  m.SampleMetadata(pose_covariance_valid=True, twist_covariance_valid=True))
        self.assertTrue(known.accepted)
        self.assertEqual(known.diagnostics['covariance']['pose']['status'], 'DECLARED_ZERO')
        self.assertFalse(known.state.valid_fields & 48)
        self.assertTrue(all(math.isnan(x) for x in known.state.pose_covariance))

    def test_covariance_nonfinite_negative_asymmetric_and_indefinite_are_rejected(self):
        matrices = []
        values = diagonal(); values[0] = math.nan; matrices.append(values)
        values = diagonal(); values[7] = -.01; matrices.append(values)
        values = diagonal(); values[1] = .001; matrices.append(values)
        values = diagonal(); values[1] = values[6] = .02; matrices.append(values)
        for values in matrices:
            with self.subTest(matrix=values):
                validator = m.Validator(config())
                raw, health = pair(); raw.pose.covariance = values
                result = validator.receive(raw, health, 1., 10060000000,
                                           m.SampleMetadata(pose_covariance_valid=True))
                self.assertFalse(result.accepted)
                self.assertEqual(result.diagnostics['covariance']['pose']['status'], 'INVALID')

    def test_covariance_psd_rank_deficiency_and_large_finite_matrix(self):
        for magnitude in (0., .01, 1.e308):
            values = [magnitude]*36
            result = m.covariance_diagnostic(values, True)
            self.assertNotEqual(result['status'], 'INVALID', result)
        tiny_indefinite = diagonal((1.e-30,)*6)
        tiny_indefinite[1] = tiny_indefinite[6] = 2.e-30
        self.assertEqual(m.covariance_diagnostic(tiny_indefinite, True)['status'], 'INVALID')

    def test_covariance_psd_checks_do_not_hide_small_indefinite_block_in_large_axis(self):
        values = diagonal((1.e308, 1.e-30, 1.e-30, .01, .01, .01))
        values[1*6+2] = values[2*6+1] = 2.e-30
        self.assertEqual(m.covariance_diagnostic(values, True)['status'], 'INVALID')
        variances = (1.e308, 1.e-308, .01, .02, .03, .04)
        roots = tuple(math.sqrt(x) for x in variances)
        rank_one = [a*b for a in roots for b in roots]
        self.assertEqual(m.covariance_diagnostic(rank_one, True)['status'], 'VALID_RAW')

    def test_explicit_covariance_caps_require_declaration_and_compare_raw_variance(self):
        validator = m.Validator(config(max_pose_variance=(.005,)*6))
        result = feed(validator)
        self.assertIn('POSE_COVARIANCE_UNKNOWN_WITH_LIMIT', result.diagnostics['reasons'])
        result = feed(validator, 2, metadata=m.SampleMetadata(pose_covariance_valid=True))
        self.assertIn('POSE_COVARIANCE_UNCERTAINTY_LIMIT_EXCEEDED', result.diagnostics['reasons'])
        validator = m.Validator(config(max_pose_variance=(.1,)*6, max_twist_variance=(.1,)*6))
        result = feed(validator, metadata=m.SampleMetadata(True, True))
        self.assertTrue(result.accepted)
        self.assertFalse(result.state.valid_fields & 48)

    def test_missing_fields_bad_version_and_bool_coercion_never_fabricate_measurements(self):
        for changes in (dict(position_valid=False), dict(velocity_valid=False),
                        dict(attitude_valid=False), dict(quality_valid=False),
                        dict(schema_version=True), dict(reset_counter=True),
                        dict(tracking='yes'), dict(quality=math.inf), dict(quality=True)):
            with self.subTest(changes=changes):
                result = feed(m.Validator(config()), **changes)
                self.assertFalse(result.accepted)
                self.assertEqual(result.state.valid_fields, 0)
                self.assertTrue(all(math.isnan(x) for x in result.state.position))

    def test_input_object_and_returned_result_cannot_mutate_current_cache(self):
        validator = m.Validator(config())
        raw, health = pair()
        result = validator.receive(raw, health, 1., 10060000000)
        result.state.position = (99., 99., 99.)
        raw.pose.pose.position.x = 99.
        health.tracking = False
        current = validator.current(1.01, 10070000000)
        self.assertTrue(current.accepted)
        self.assertEqual(current.state.position, (0., 0., 0.))

    def test_returned_covariance_diagnostic_cannot_mutate_cached_diagnostic(self):
        validator = m.Validator(config())
        self.assertTrue(feed(validator).accepted)
        first = validator.current(1.06, 10070000000)
        first.diagnostics['covariance']['pose']['status'] = 'FORGED_BY_CALLER'
        first.diagnostics['covariance']['pose']['reasons'].append('FORGED_REASON')
        second = validator.current(1.07, 10080000000)
        self.assertTrue(second.accepted)
        self.assertEqual(second.diagnostics['covariance']['pose']['status'], 'UNKNOWN')
        self.assertEqual(second.diagnostics['covariance']['pose']['reasons'], [])

    def test_event_queue_is_bounded_and_healthy_tail_keeps_anomaly_counts(self):
        validator = m.Validator(config())
        feed(validator, tracking=False)
        for sequence in range(2, 160):
            feed(validator, sequence)
        current = validator.current(1.+159*.05+.01, 10000000000+159*50000000+20000000)
        self.assertTrue(current.accepted)
        self.assertGreater(current.diagnostics['counters']['events_dropped'], 0)
        self.assertIsNotNone(current.diagnostics['last_anomaly'])
        self.assertLessEqual(len(validator.drain_events()), 128)
        self.assertEqual(validator.drain_events(), [])

    def test_invalid_configuration_is_rejected_without_ros(self):
        for changes in (dict(receive_timeout_sec=0), dict(max_source_age_sec=-1),
                        dict(max_pose_variance=(.1,)), dict(max_twist_variance=1),
                        dict(min_quality=1.1), dict(max_position_step_m=math.nan),
                        dict(min_quality=.8, nominal_quality_min=.4),
                        dict(epoch_history_capacity=0), dict(epoch_history_capacity=True),
                        dict(epoch_history_capacity=4097)):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                config(**changes)
        with self.assertRaises(ValueError):
            m.ClockPolicy(confirmed=True)
        with self.assertRaises(ValueError):
            m.ClockPolicy(offset_ns=True)
        with self.assertRaises(ValueError):
            m.SampleMetadata(pose_covariance_valid='true')

    def test_config_copies_transform_lists_before_caller_can_change_them(self):
        transforms = (
            ('body_from_sensor_wxyz', [1, 0, 0, 0], [0., 0., 0., 1.]),
            ('sensor_position_body_frd', [0, 0, 0], [1., 2., 3.]),
            ('world_alignment_wxyz', [1, 0, 0, 0], [0., 0., 0., 1.]),
            ('world_translation_ned', [0, 0, 0], [100., 200., 300.]),
        )
        motion = dict(position=(1., 2., 3.), velocity=(.4, .5, .6),
                      omega=(.1, .2, .3))
        for field, original, changed in transforms:
            with self.subTest(field=field):
                supplied = list(original)
                configured = config(adapter=adapter_config(**{field: supplied}))
                captured = getattr(configured.adapter, field)
                self.assertIsInstance(captured, tuple)
                self.assertTrue(all(type(value) is float for value in captured))
                validator = m.Validator(configured)
                first = feed(validator, **motion)
                self.assertTrue(first.accepted)

                # 创建配置后修改来源列表，不得悄悄更换可信周期内使用的变换。
                supplied[:] = changed
                second = feed(validator, 2, **motion)
                self.assertTrue(second.accepted, second.diagnostics['reasons'])
                self.assertEqual(getattr(configured.adapter, field), captured)
                for name in ('position', 'velocity', 'attitude', 'angular_velocity'):
                    self.assertEqual(getattr(second.state, name), getattr(first.state, name))


if __name__ == '__main__':
    unittest.main()
