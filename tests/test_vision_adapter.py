"""Real odometry conversion and ROS wire-to-core mapping contracts; no ROS graph."""
import importlib.util
import math
from pathlib import Path
import subprocess
import tempfile
from types import SimpleNamespace as NS
import unittest

ROOT = Path(__file__).resolve().parents[1]
ADAPTER = ROOT / 'native/tools/vision_odometry_adapter.py'


def stamp(sec=10, nanosec=25):
    return NS(sec=sec, nanosec=nanosec)


def odometry(position=(5., 6., 7.), velocity=(0., 0., 0.),
             q=(1., 0., 0., 0.), omega=(0., 0., 0.)):
    def vector(v):
        return NS(x=v[0], y=v[1], z=v[2])
    return NS(header=NS(stamp=stamp(), frame_id='vendor_odom'), child_frame_id='vendor_sensor',
              pose=NS(pose=NS(position=vector(position),
                              orientation=NS(w=q[0], x=q[1], y=q[2], z=q[3])),
                      covariance=[0.] * 36),
              twist=NS(twist=NS(linear=vector(velocity), angular=vector(omega)), covariance=[0.] * 36))


def health(**changes):
    fields = dict(schema_version=1, stamp=stamp(), sequence=7, source_id='fixture-session-1',
                  frame_id='vendor_odom', body_frame_id='vendor_sensor', reset_counter=4,
                  reset_valid=True, initialized=True, tracking=True, calibration_valid=True,
                  frame_alignment_valid=True, time_sync_valid=True, extrinsics_valid=True,
                  metric_scale_valid=True, position_valid=True, velocity_valid=True,
                  attitude_valid=True, angular_velocity_valid=True,
                  angular_velocity_source='fixture_gyro', quality_valid=True, quality=.9,
                  extrinsics_id='mount-v1', alignment_id='align-v1')
    fields.update(changes)
    return NS(**fields)


class OdometryAdapterTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.adapter = None
        if ADAPTER.is_file():
            spec = importlib.util.spec_from_file_location('fc01_test_vision_adapter', ADAPTER)
            cls.adapter = importlib.util.module_from_spec(spec)
            # dataclasses accesses the current module during class creation.
            import sys
            sys.modules[spec.name] = cls.adapter
            spec.loader.exec_module(cls.adapter)

    def setUp(self):
        self.assertTrue(ADAPTER.is_file(), 'vision odometry adapter implementation is missing')
        self.a = self.adapter
        self.config = self.a.AdapterConfig(
            source_id='fixture-session-1', input_frame_id='vendor_odom',
            input_body_frame_id='vendor_sensor', world_convention='NED', body_convention='FRD',
            body_from_sensor_wxyz=(1., 0., 0., 0.), sensor_position_body_frd=(0., 0., 0.),
            world_alignment_wxyz=(1., 0., 0., 0.), world_translation_ned=(0., 0., 0.),
            extrinsics_id='mount-v1', alignment_id='align-v1')

    def convert(self, msg=None, meta=None, **config_changes):
        from dataclasses import replace
        config = replace(self.config, **config_changes)
        return self.a.convert_odometry(msg or odometry(), meta or health(), config)

    def assertVector(self, actual, expected):
        for x, y in zip(actual, expected):
            self.assertAlmostEqual(x, y, places=10)

    def test_ned_frd_preserves_explicit_valid_fields_and_wxyz(self):
        state = self.convert()
        self.assertEqual(state.reason, 'OK')
        self.assertEqual(state.stamp_ns, 10_000_000_025)
        self.assertEqual(state.sequence, 7)
        self.assertEqual(state.reset_counter, 4)
        self.assertEqual(state.frame_id, 'fc01_vision_ned')
        self.assertEqual(state.body_frame_id, 'base_link_frd')
        self.assertVector(state.position, (5., 6., 7.))
        self.assertVector(state.attitude, (1., 0., 0., 0.))
        self.assertEqual(state.valid_fields, 1 | 2 | 4 | 8 | 64)
        self.assertTrue(all(math.isnan(x) for x in state.pose_covariance + state.twist_covariance))

    def test_enu_flu_rotates_pose_child_twist_and_angular_velocity(self):
        h = math.sqrt(.5)
        state = self.convert(odometry((1., 2., 3.), (1., 2., 3.), (h, 0., 0., h), (0., 0., 2.)),
                             world_convention='ENU', body_convention='FLU')
        self.assertEqual(state.reason, 'OK')
        self.assertVector(state.position, (2., 1., -3.))
        self.assertVector(state.velocity, (1., -2., -3.))
        self.assertVector(state.attitude, (1., 0., 0., 0.))
        self.assertVector(state.angular_velocity, (0., 0., -2.))

    def test_lever_arm_changes_position_and_removes_rotational_sensor_velocity(self):
        state = self.convert(odometry(velocity=(0., 2., 0.), omega=(0., 0., 2.)),
                             sensor_position_body_frd=(1., 0., 0.))
        self.assertVector(state.position, (4., 6., 7.))
        self.assertVector(state.velocity, (0., 0., 0.))

    def test_nonzero_lever_arm_without_angular_measurement_invalidates_velocity(self):
        state = self.convert(meta=health(angular_velocity_valid=False),
                             sensor_position_body_frd=(1., 0., 0.))
        self.assertFalse(state.valid_fields & 2)
        self.assertFalse(state.valid_fields & 8)
        self.assertTrue(all(math.isnan(x) for x in state.velocity))
        self.assertTrue(all(math.isnan(x) for x in state.angular_velocity))
        self.assertNotEqual(state.reason, 'OK')

    def test_rotated_mount_and_world_alignment_are_applied(self):
        h = math.sqrt(.5)
        state = self.convert(body_from_sensor_wxyz=(h, 0., 0., h),
                             sensor_position_body_frd=(1., 0., 0.),
                             world_alignment_wxyz=(h, 0., 0., h),
                             world_translation_ned=(1., 2., 3.))
        self.assertVector(state.position, (-6., 7., 10.))
        self.assertVector(state.attitude, (1., 0., 0., 0.))

    def test_missing_or_mismatched_health_never_makes_zero_values_valid(self):
        for meta in (None, health(stamp=stamp(10, 26)), health(frame_id='other'),
                     health(body_frame_id='other'), health(source_id='other')):
            with self.subTest(meta=meta):
                state = self.a.convert_odometry(odometry(), meta, self.config)
                self.assertEqual(state.valid_fields, 0)
                self.assertFalse(state.calibration_valid)
                self.assertTrue(all(math.isnan(x) for x in state.position))

    def test_each_quality_time_reset_and_calibration_contract_is_required(self):
        for field in ('reset_valid', 'initialized', 'tracking', 'calibration_valid',
                      'frame_alignment_valid', 'time_sync_valid', 'extrinsics_valid',
                      'metric_scale_valid', 'quality_valid'):
            with self.subTest(field=field):
                state = self.convert(meta=health(**{field: False}))
                self.assertEqual(state.valid_fields, 0)
        for value in (float('nan'), -1., 1.1):
            with self.subTest(quality=value):
                self.assertEqual(self.convert(meta=health(quality=value)).valid_fields, 0)
        for values in ({'extrinsics_id': 'new'}, {'alignment_id': 'new'}, {'schema_version': 2}):
            self.assertEqual(self.convert(meta=health(**values)).valid_fields, 0)

    def test_zero_quality_remains_explicit_measured_quality_for_core_threshold(self):
        state = self.convert(meta=health(quality=0.))
        self.assertEqual(state.quality, 0.)
        self.assertTrue(state.valid_fields & 64)

    def test_absent_position_velocity_attitude_and_angular_source_are_not_invented(self):
        for field, bit, array in (('position_valid', 1, 'position'), ('velocity_valid', 2, 'velocity'),
                                  ('attitude_valid', 4, 'attitude')):
            state = self.convert(meta=health(**{field: False}))
            self.assertFalse(state.valid_fields & bit)
            self.assertTrue(all(math.isnan(x) for x in getattr(state, array)))
        state = self.convert(meta=health(angular_velocity_source=''))
        self.assertFalse(state.valid_fields & 8)

    def test_nonfinite_or_zero_norm_raw_measurements_are_rejected(self):
        self.assertFalse(self.convert(odometry(position=(math.nan, 0., 0.))).valid_fields & 1)
        self.assertFalse(self.convert(odometry(velocity=(math.inf, 0., 0.))).valid_fields & 2)
        self.assertFalse(self.convert(odometry(q=(0., 0., 0., 0.))).valid_fields & 4)
        self.assertFalse(self.convert(odometry(q=(2., 0., 0., 0.))).valid_fields & 4)
        self.assertEqual(self.convert(odometry(q=(1.01, 0., 0., 0.))).reason, 'OK')

    def test_transform_overflow_never_marks_nonfinite_derived_measurements_valid(self):
        state = self.convert(odometry(position=(1.e308, 0., 0.)),
                             world_translation_ned=(1.e308, 0., 0.))
        self.assertFalse(state.valid_fields & 1)
        self.assertTrue(all(math.isnan(x) for x in state.position))
        half_yaw = math.pi / 8.
        state = self.convert(odometry(velocity=(1.7e308, 1.7e308, 0.),
                                      q=(math.cos(half_yaw), 0., 0., math.sin(half_yaw))))
        self.assertFalse(state.valid_fields & 2)
        self.assertTrue(all(math.isnan(x) for x in state.velocity))
        state = self.convert(odometry(omega=(1.7e308, 1.7e308, 0.)),
                             body_from_sensor_wxyz=(math.cos(half_yaw), 0., 0., math.sin(half_yaw)))
        self.assertFalse(state.valid_fields & 8)
        self.assertTrue(all(math.isnan(x) for x in state.angular_velocity))

    def test_half_turn_orientation_conversion_is_stable_on_each_axis(self):
        for quaternion in ((0., 1., 0., 0.), (0., 0., 1., 0.), (0., 0., 0., 1.)):
            with self.subTest(quaternion=quaternion):
                state = self.convert(odometry(q=quaternion))
                self.assertVector(state.attitude, quaternion)
                self.assertEqual(state.reason, 'OK')

    def test_bad_stamp_is_invalid_and_config_needs_explicit_conventions(self):
        for raw in (stamp(-1), stamp(0, 0), stamp(10, 1_000_000_000)):
            msg = odometry()
            msg.header.stamp = raw
            self.assertEqual(self.convert(msg).valid_fields, 0)
        from dataclasses import replace
        for changes in ({'source_id': ''}, {'extrinsics_id': ''}, {'world_convention': ''},
                        {'body_convention': 'OPTICAL'}, {'body_from_sensor_wxyz': (0., 0., 0., 0.)}):
            with self.assertRaises(ValueError):
                replace(self.config, **changes).validate()

    def test_stamp_pair_buffer_matches_exact_sample_and_never_republishes_duplicates(self):
        pairs = self.a.StampPairBuffer(timeout=.05, max_pending=4)
        self.assertEqual(pairs.odometry(odometry(), 1.), [])
        ready = pairs.health(health(), 1.01)
        self.assertEqual(len(ready), 1)
        self.assertIsNotNone(ready[0][1])
        self.assertEqual(pairs.health(health(), 1.02), [])
        self.assertEqual(pairs.odometry(odometry(), 1.03), [])
        later = odometry()
        later.header.stamp = stamp(10, 30)
        self.assertEqual(pairs.odometry(later, 2.), [])
        expired = pairs.expire(2.06)
        self.assertEqual(len(expired), 1)
        self.assertIsNone(expired[0][1])
        self.assertEqual(pairs.health(health(stamp=stamp(10, 30)), 2.07), [])

    def guarded(self):
        self.assertTrue(hasattr(self.a, 'GuardedOdometryAdapter'),
                        'raw source graph authority must guard the exact-stamp adapter')
        return self.a.GuardedOdometryAdapter(self.config, pair_timeout=.05, max_pending=4)

    def test_late_health_cannot_rescue_odometry_at_or_after_pair_timeout(self):
        for delay in (.05, .20):
            with self.subTest(delay=delay):
                pairs = self.a.StampPairBuffer(timeout=.05, max_pending=4)
                self.assertEqual(pairs.odometry(odometry(), 0.), [])
                # 达到窗口边界即先撤销旧测量，不能被同时间戳的晚到健康声明恢复。
                expired = pairs.health(health(), delay)
                self.assertEqual(len(expired), 1)
                self.assertIsNone(expired[0][1])
                self.assertEqual(pairs.health(health(), delay + .01), [])
                self.assertEqual(pairs.odometry(odometry(), delay + .02), [])

    def test_late_odometry_cannot_borrow_health_at_or_after_pair_timeout(self):
        for delay in (.05, .20):
            with self.subTest(delay=delay):
                pairs = self.a.StampPairBuffer(timeout=.05, max_pending=4)
                self.assertEqual(pairs.health(health(), 0.), [])
                # 先到的声明已经过期；新到的原始测量应重新等待自己的健康配对。
                self.assertEqual(pairs.odometry(odometry(), delay), [])
                expired = pairs.expire(delay + .06)
                self.assertEqual(len(expired), 1)
                self.assertIsNone(expired[0][1])

    def test_timely_pair_remains_accepted_in_either_callback_order(self):
        for health_first in (True, False):
            with self.subTest(health_first=health_first):
                pairs = self.a.StampPairBuffer(timeout=.05, max_pending=4)
                first = pairs.health if health_first else pairs.odometry
                second = pairs.odometry if health_first else pairs.health
                self.assertEqual(first(health() if health_first else odometry(), 0.), [])
                ready = second(odometry() if health_first else health(), .049)
                self.assertEqual(len(ready), 1)
                self.assertIsNotNone(ready[0][1])

    def test_callback_expiry_precedes_fresh_pair_without_hiding_revocation(self):
        adapter = self.guarded()
        self.assertEqual(adapter.odometry(odometry(), 1., 10_000_000_050, 1, 1), [])
        later = odometry()
        later.header.stamp = stamp(10, 400)
        later_health = health(stamp=stamp(10, 400), sequence=8)
        self.assertEqual(adapter.health(later_health, 1.04, 10_000_000_450, 1, 1), [])

        # 定时器尚未执行：这次回调必须同时保留旧样本撤销和新样本转换，按此顺序返回。
        states = adapter.odometry(later, 1.06, 10_000_000_500, 1, 1)
        self.assertEqual(len(states), 2)
        self.assertEqual(states[0].valid_fields, 0)
        self.assertEqual(states[0].reason, 'MISSING_HEALTH')
        self.assertEqual(states[1].reason, 'OK')
        self.assertGreater(states[1].sequence, states[0].sequence)
        self.assertEqual(states[1].raw_health_sequence, 8)

    def test_raw_topics_each_require_exactly_one_publisher_before_conversion(self):
        for counts in ((0, 1), (1, 0), (2, 1), (1, 2), (2, 2)):
            with self.subTest(counts=counts):
                adapter = self.guarded()
                states = adapter.health(health(), 1., 10_000_000_100, *counts)
                self.assertEqual(len(states), 1)
                invalid = states[0]
                self.assertEqual(invalid.valid_fields, 0)
                self.assertFalse(invalid.tracking or invalid.initialized or invalid.calibration_valid)
                self.assertEqual(invalid.source_id, 'fixture-session-1')
                self.assertEqual(invalid.stamp_ns, 10_000_000_100)
                self.assertGreater(invalid.sequence, 0)
                self.assertTrue(all(math.isnan(x) for x in invalid.position + invalid.velocity + invalid.attitude))

    def test_raw_conflict_discards_cached_health_before_another_odometry_can_pair(self):
        adapter = self.guarded()
        self.assertEqual(adapter.health(health(), 1., 10_000_000_050, 1, 1), [])
        invalid = adapter.odometry(odometry(), 1.01, 10_000_000_100, 2, 1)[0]
        self.assertEqual(invalid.valid_fields, 0)
        # The former health from A cannot bless the remaining raw Odom from B.
        self.assertEqual(adapter.odometry(odometry(), 1.02, 10_000_000_110, 1, 1), [])
        missing = adapter.poll(1.08, 10_000_000_200, 1, 1)
        self.assertEqual(len(missing), 1)
        self.assertEqual(missing[0].valid_fields, 0)
        self.assertGreater(missing[0].sequence, invalid.sequence)

    def test_timer_revokes_healthy_state_on_raw_graph_conflict_and_waits_for_new_pair(self):
        adapter = self.guarded()
        adapter.health(health(), 1., 10_000_000_050, 1, 1)
        valid = adapter.odometry(odometry(), 1.01, 10_000_000_100, 1, 1)[0]
        self.assertEqual(valid.reason, 'OK')
        self.assertEqual(valid.raw_health_sequence, 7)
        invalid = adapter.poll(1.02, 10_000_000_200, 1, 2)[0]
        self.assertEqual(invalid.valid_fields, 0)
        self.assertGreater(invalid.sequence, valid.sequence)
        self.assertGreaterEqual(invalid.stamp_ns, valid.stamp_ns)
        self.assertEqual(adapter.poll(1.03, 10_000_000_300, 1, 1), [])
        later_odom = odometry()
        later_odom.header.stamp = stamp(10, 400)
        later_health = health(stamp=stamp(10, 400), sequence=8)
        self.assertEqual(adapter.health(later_health, 1.04, 10_000_000_450, 1, 1), [])
        restored = adapter.odometry(later_odom, 1.05, 10_000_000_500, 1, 1)[0]
        self.assertEqual(restored.reason, 'OK')
        self.assertGreater(restored.sequence, invalid.sequence)
        self.assertEqual(restored.raw_health_sequence, 8)

    def test_missing_metadata_emits_explicit_revocation_without_borrowing_health(self):
        adapter = self.guarded()
        adapter.health(health(), 1., 10_000_000_050, 1, 1)
        valid = adapter.odometry(odometry(), 1.01, 10_000_000_100, 1, 1)[0]
        later = odometry()
        later.header.stamp = stamp(10, 200)
        adapter.odometry(later, 1.02, 10_000_000_250, 1, 1)
        invalid = adapter.poll(1.08, 10_000_000_300, 1, 1)[0]
        self.assertEqual(invalid.valid_fields, 0)
        self.assertFalse(invalid.tracking or invalid.calibration_valid)
        self.assertGreater(invalid.sequence, valid.sequence)
        self.assertGreaterEqual(invalid.stamp_ns, valid.stamp_ns)


class RosMappingTests(unittest.TestCase):
    def test_cpp_mapping_preserves_wxyz_masks_mode_and_rejects_invalid_stamp_version(self):
        header = ROOT / 'native/src/jikl_fc01_system/include/jikl_fc01_system/vision_ros_adapter.hpp'
        self.assertTrue(header.is_file(), 'C++ vision wire adapter implementation is missing')
        source = r'''
#include "jikl_fc01_system/vision_ros_adapter.hpp"
#include <cassert>
struct Stamp {int sec{10}; unsigned nanosec{25};};
struct StateMessage {
 unsigned short schema_version{1},valid_fields{69};
 unsigned long sequence{7}; Stamp stamp; unsigned reset_counter{4};
 std::string frame_id{"fc01_vision_ned"},body_frame_id{"base_link_frd"},source_id{"fixture"},angular_velocity_source{"gyro"};
 bool initialized{true},tracking{true},calibration_valid{true};
 std::array<double,3> position{1,2,3},velocity{4,5,6},angular_velocity{7,8,9};
 std::array<double,4> attitude{.5,.5,.5,.5};
 std::array<double,36> pose_covariance{},twist_covariance{};double quality{.9};
};
struct CommandMessage {
 unsigned short schema_version{1}; unsigned long sequence{9}; Stamp stamp; unsigned reset_counter{4};
 std::string source_id{"planner"},frame_id{"fc01_vision_ned"};unsigned char mode{2};
 bool valid{true},position_valid{true},yaw_valid{true},sticks_valid{false};
 std::array<double,3> position{1,2,3};double yaw{.4};std::array<double,4> sticks{};
};
int main(){
 StateMessage message;auto state=fc01::vision::state_from_message(message);
 assert(state.sequence==7&&state.stamp_ns==10000000025ULL&&state.reset_counter==4);
 assert(state.received_at==-1&&state.attitude[0]==.5&&state.valid_fields==69);
 assert(state.position[2]==3&&state.angular_velocity_source=="gyro"&&state.quality==.9);
 message.schema_version=2;assert(fc01::vision::state_from_message(message).valid_fields==0);
 message.schema_version=1;message.stamp.sec=-1;assert(fc01::vision::state_from_message(message).valid_fields==0);
 message.stamp.sec=10;message.stamp.nanosec=1000000000;assert(fc01::vision::state_from_message(message).valid_fields==0);
 CommandMessage target;auto command=fc01::vision::command_from_message(target);
 assert(command.mode==fc01::vision::Mode::Cruise&&command.valid&&command.position_valid);
 assert(command.stamp_ns==10000000025ULL&&command.received_at==-1&&command.yaw==.4);
 target.mode=255;assert(!fc01::vision::command_from_message(target).valid);
 target.mode=2;target.schema_version=2;assert(!fc01::vision::command_from_message(target).valid);
 target.schema_version=1;target.stamp.sec=-1;assert(!fc01::vision::command_from_message(target).valid);
}
'''
        with tempfile.TemporaryDirectory(prefix='fc01_vision_wire_') as directory:
            cpp = Path(directory) / 'wire.cpp'
            binary = Path(directory) / 'wire'
            cpp.write_text(source)
            result = subprocess.run(['g++', '-std=c++17', '-Wall', '-Wextra', '-Werror', '-pedantic',
                                     '-I', str(header.parents[1]), str(cpp), '-o', str(binary)],
                                    capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            subprocess.run([str(binary)], check=True)


if __name__ == '__main__':
    unittest.main()
