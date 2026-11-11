# 独立速度接收器与有界证据回归：纯主机消息替身，不启动ROS节点或控制器。
# 分别验证观察历史、可用候选和控制权限：即使显式策略使缓存可用，control_authority仍必须为false。
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
from unittest.mock import patch


SCRIPT = Path(__file__).resolve().parents[1] / 'native/tools/velocity_command_diagnostic.py'
SPEC = importlib.util.spec_from_file_location('fc01_velocity_diagnostic_test', SCRIPT)
diagnostic = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = diagnostic
SPEC.loader.exec_module(diagnostic)


# 生成body_frd合成目标速度，默认测量戳随序号前进；按关键字覆盖以制造单一错误。
def message(sequence=1, **changes):
    fields = dict(schema_version=1, sequence=sequence,
                  stamp=NS(sec=100, nanosec=sequence * 1000000),
                  reset_counter=0, source_id='synthetic-session-1', frame_id='body_frd',
                  valid=True, vx=.2, vy=-.1, vz=-.05)
    fields.update(changes)
    return NS(**fields)


# 接收器边界：字段、双时钟、重放高水位、源唯一性、撤销及异常证据都独立验证。
class ReceiverTests(unittest.TestCase):
    # 仅把本用例显式提供的限制交给DiagnosticConfig；不代填生产策略。
    def receiver(self, **limits):
        return diagnostic.Receiver(diagnostic.DiagnosticConfig(**limits))

    # 默认允许记录载荷，但阈值未配置、新鲜度未知、无候选且无控制权限，不能凭收到包就导航。
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

    # 仅设置接收超时可判定本机收包年龄，仍不能确认源时钟或命令整体新鲜度。
    def test_configured_receive_timeout_is_not_a_source_clock_qualification(self):
        receiver = self.receiver(timeout_sec=.2)
        receiver.receive(message(), 10., 100001000000, 1)
        state = receiver.snapshot(10.1, 100101000000)
        self.assertEqual(state['receive_freshness'], 'WITHIN_RECEIVE_LIMIT')
        self.assertEqual(state['freshness'], 'UNCONFIGURED')

    # 源年龄和未来容差显式给定后可报告CURRENT，但历史有效包仍不取得飞行权限。
    def test_explicit_clock_limits_can_report_current_while_authority_stays_false(self):
        receiver = self.receiver(timeout_sec=.2, max_source_age_sec=.2, future_tolerance_sec=.01)
        receiver.receive(message(), 10., 100001000000, 1)
        state = receiver.snapshot(10.1, 100101000000)
        self.assertEqual(state['freshness'], 'CURRENT')
        self.assertFalse(state['control_authority'])
        self.assertTrue(state['last_valid_is_historical'])

    # 重复与倒序序号不能刷新last_valid时间；过期只计一次，不能靠重放续命。
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

    # 新invalid包须立即撤销最新可用观察；恢复后latest_errors清空，但last_anomaly事件号保留。
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

    # 复位和来源变化分别记录RESET_CHANGED/SOURCE_CHANGED，不与飞机自身状态或权限绑定。
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

    # 同一来源身份序号从100回到1必须报倒序，不能把进程重启当作新可信会话。
    def test_restart_without_new_identity_is_visible_and_not_silently_accepted(self):
        receiver = self.receiver()
        receiver.receive(message(100), 10., 100100000000, 1)
        event = receiver.receive(message(1), 10.1, 100101000000, 1)
        self.assertIn('OUT_OF_ORDER_SEQUENCE', event['reasons'])
        self.assertAlmostEqual(receiver.snapshot(10.1, 100101000000)['last_valid_age_sec'], .1)

    # 从单源变双源再恢复，冲突计数和异常原因仍须保留。
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

    # 空/错误坐标系、版本、序号、来源、时间或非有限速度均拒绝；JSON用null表示无效值。
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

    # 速度限制比较三维向量模长，另分别检查源样本过旧及过于未来。
    def test_explicit_vector_speed_and_clock_limits_are_checked(self):
        receiver = self.receiver(max_speed_mps=.5, max_source_age_sec=.1, future_tolerance_sec=.02)
        event = receiver.receive(message(vx=.4, vy=.4, vz=0.), 10., 100201000000, 1)
        self.assertIn('SPEED_LIMIT_EXCEEDED', event['reasons'])
        self.assertIn('SOURCE_AGE_LIMIT_EXCEEDED', event['reasons'])
        event = self.receiver(future_tolerance_sec=.02).receive(message(), 10., 99900000000, 1)
        self.assertIn('SOURCE_STAMP_IN_FUTURE', event['reasons'])

    # 只换序号而复用测量戳必须报STAMP_NOT_ADVANCING，不能刷新有效年龄。
    def test_same_stamp_cannot_be_reissued_under_a_new_sequence(self):
        receiver = self.receiver()
        receiver.receive(message(), 10., 100001000000, 1)
        event = receiver.receive(message(2, stamp=NS(sec=100, nanosec=1000000)),
                                 10.1, 100101000000, 1)
        self.assertIn('STAMP_NOT_ADVANCING', event['reasons'])
        self.assertAlmostEqual(receiver.snapshot(10.1, 100101000000)['last_valid_age_sec'], .1)

    # 连续400包后事件队列限定128条，丢弃数量必须公开。
    def test_event_memory_is_bounded_and_drops_are_counted(self):
        receiver = self.receiver()
        for index in range(400):
            receiver.receive(message(index + 1), 10. + index / 100., 100001000000 + index * 1000000, 1)
        self.assertEqual(len(receiver.drain_events()), 128)
        self.assertGreater(receiver.counters['events_dropped'], 0)

    # 负数、非有限数、布尔值和超大整数阈值须抛ValueError，不退回隐式默认。
    def test_invalid_configuration_rejected_without_implicit_defaults(self):
        for value in (-1., math.nan, math.inf, True, 10**400):
            with self.subTest(value=value), self.assertRaises(ValueError):
                diagnostic.DiagnosticConfig(timeout_sec=value)

    # 超大接收时钟在接收或快照入口都须显式拒绝，received计数保持0。
    def test_oversized_integer_clocks_raise_explicit_validation_error(self):
        receiver = self.receiver()
        for action in (lambda: receiver.receive(message(), 10**400, 100001000000, 1),
                       lambda: receiver.snapshot(10**400, 100001000000)):
            with self.subTest(action=action), self.assertRaises(ValueError):
                action()
        self.assertEqual(receiver.counters['received'], 0)

    # 超大整数速度不能中断接收器；拒绝后缓存失效，后续正常样本可恢复。
    def test_oversized_integer_velocity_is_rejected_without_interrupting_receiver(self):
        receiver = self.qualified()
        event = receiver.receive(message(vx=10**400), 10., 100001000000, 1)
        self.assertFalse(event['accepted'])
        self.assertIn('VELOCITY_NONFINITE_OR_MISSING', event['reasons'])
        state = receiver.snapshot(10.01, 100011000000)
        self.assertFalse(state['cache_usable'])
        json.dumps(state, allow_nan=False)
        recovered = receiver.receive(message(), 10.02, 100021000000, 1)
        self.assertTrue(recovered['accepted'])
        self.assertTrue(receiver.snapshot(10.03, 100031000000)['cache_usable'])

    # 调用者修改返回事件的嵌套理由、速度和stamp，不得污染保留异常或待取队列。
    def test_returned_event_cannot_modify_retained_anomaly(self):
        receiver = self.receiver()
        event = receiver.receive(message(valid=False), 10., 100001000000, 1)
        event_id = event['event_id']
        event['reasons'].clear()
        event['sample']['vx'] = 99.
        event['sample']['stamp']['sec'] = 999

        state = receiver.snapshot(10., 100001000000)
        for key in ('last_event', 'last_anomaly'):
            self.assertEqual(state[key]['event_id'], event_id)
            self.assertEqual(state[key]['reasons'], ['COMMAND_INVALID'])
            self.assertEqual(state[key]['sample']['vx'], .2)
            self.assertEqual(state[key]['sample']['stamp']['sec'], 100)
        queued = receiver.drain_events()[-1]
        self.assertEqual(queued, state['last_anomaly'])

    # 取走队列后修改嵌套对象，也不能回写last_event/last_anomaly；队列再次取出应为空。
    def test_drained_events_cannot_modify_retained_anomaly(self):
        receiver = self.receiver()
        receiver.receive(message(valid=False), 10., 100001000000, 1)
        events = receiver.drain_events()
        self.assertEqual([event['kind'] for event in events], ['publisher_count', 'message'])
        event = events[-1]
        event_id = event['event_id']
        event['reasons'].clear()
        event['sample']['vx'] = 99.
        event['sample']['stamp']['sec'] = 999

        state = receiver.snapshot(10., 100001000000)
        for key in ('last_event', 'last_anomaly'):
            self.assertEqual(state[key]['event_id'], event_id)
            self.assertEqual(state[key]['reasons'], ['COMMAND_INVALID'])
            self.assertEqual(state[key]['sample']['vx'], .2)
            self.assertEqual(state[key]['sample']['stamp']['sec'], 100)
        self.assertEqual(receiver.drain_events(), [])
        self.assertEqual(state['counters']['anomalies'], 1)

    # 给出完整合成策略和绑定来源，便于区别“记录到消息”与“当前缓存可用”。
    def qualified(self):
        return self.receiver(timeout_sec=.3, max_speed_mps=.5, max_source_age_sec=.2,
                             future_tolerance_sec=.01, expected_source_id='synthetic-session-1')

    # invalid立即清当前候选，同时保留历史接受序号；新正常包可重新形成候选。
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

    # 来源数为0或2须撤销候选；仅恢复为1不够，须收到新的可信包。
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

    # 没有新包时也重算源年龄，过期清候选并计数；后续推进测量戳的包可恢复。
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

    # 本机单调时钟回退后不能靠小幅前进恢复，须回到原高水位；拒绝序号不污染可信记录。
    def test_receive_clock_regression_recovers_only_after_previous_high_watermark(self):
        receiver = self.qualified()
        receiver.receive(message(), 10., 100001000000, 1)
        state = receiver.snapshot(9.9, 100002000000)
        self.assertFalse(state['cache_usable'])
        self.assertIn('RECEIVE_CLOCK_REGRESSION', state['last_anomaly']['reasons'])
        rejected = receiver.receive(message(2), 9.91, 100003000000, 1)
        self.assertFalse(rejected['accepted'])
        self.assertIn('RECEIVE_CLOCK_REGRESSION', rejected['reasons'])
        state = receiver.snapshot(9.92, 100004000000)
        self.assertFalse(state['cache_usable'])
        self.assertEqual(state['historical_last_accepted_message']['sequence'], 1)

        # 回到原时间后允许新消息恢复；刚才被拒绝的序号没有污染可信记录。
        recovered = receiver.receive(message(2), 10., 100005000000, 1)
        self.assertTrue(recovered['accepted'])
        self.assertTrue(receiver.snapshot(10.01, 100006000000)['cache_usable'])

    # ROS时钟回退须撤销候选并保留原高水位，追平之前新包仍拒绝。
    def test_ros_clock_regression_recovers_only_after_previous_high_watermark(self):
        receiver = self.qualified()
        receiver.receive(message(), 10., 100100000000, 1)
        state = receiver.snapshot(10.01, 100000000000)
        self.assertFalse(state['cache_usable'])
        self.assertIn('ROS_CLOCK_REGRESSION', state['last_anomaly']['reasons'])
        rejected = receiver.receive(message(2), 10.02, 100002000000, 1)
        self.assertFalse(rejected['accepted'])
        self.assertIn('ROS_CLOCK_REGRESSION', rejected['reasons'])
        state = receiver.snapshot(10.03, 100003000000)
        self.assertFalse(state['cache_usable'])
        self.assertEqual(state['historical_last_accepted_message']['sequence'], 1)
        self.assertEqual(receiver.last_observed_ros_ns, 100100000000)

        recovered = receiver.receive(message(2, stamp=NS(sec=100, nanosec=100000000)),
                                     10.04, 100100000000, 1)
        self.assertTrue(recovered['accepted'])
        self.assertTrue(receiver.snapshot(10.05, 100101000000)['cache_usable'])

    # 源测量戳倒退即使序号增加也须拒绝，保留原历史接受包。
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

    # 绑定会话之外的来源可作为异常观察保存，但不得覆盖历史可信来源或形成候选。
    def test_bound_source_mismatch_never_becomes_a_current_candidate(self):
        receiver = self.qualified()
        receiver.receive(message(), 10., 100001000000, 1)
        event = receiver.receive(message(2, source_id='unexpected-session'), 10.01, 100011000000, 1)
        self.assertIn('SOURCE_ID_NOT_EXPECTED', event['reasons'])
        state = receiver.snapshot(10.02, 100021000000)
        self.assertFalse(state['cache_usable'])
        self.assertEqual(state['last_message']['source_id'], 'unexpected-session')
        self.assertEqual(state['historical_last_accepted_message']['source_id'], 'synthetic-session-1')

    # 缺少速度限制等显式策略时，即使其余时序配置齐全也不得让缓存可用。
    def test_unbound_source_or_unconfigured_speed_cannot_qualify_cache(self):
        receiver = self.receiver(timeout_sec=.2, max_source_age_sec=.2, future_tolerance_sec=.01,
                                 expected_source_id='synthetic-session-1')
        receiver.receive(message(), 10., 100001000000, 1)
        state = receiver.snapshot(10.01, 100011000000)
        self.assertFalse(state['cache_usable'])
        self.assertIn('LIMITS_UNCONFIGURED', state['cache_errors'])

    # 来源A→错误B→A的旧包仍是重复，不能利用中间来源变化重置重放证据。
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

    # 以最大序号附带未来戳/坏版本/坏帧/NaN/巨速，拒绝后正常序号2仍可恢复。
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

    # valid=false的有效撤销占用该序号；同序号改回true仍是重放，须新序号恢复。
    def test_explicit_withdrawal_cannot_be_reissued_with_same_sequence_as_valid(self):
        receiver = self.qualified()
        receiver.receive(message(), 10., 100001000000, 1)
        receiver.receive(message(2, valid=False), 10.01, 100011000000, 1)
        replay = receiver.receive(message(2), 10.02, 100021000000, 1)
        self.assertIn('DUPLICATE_SEQUENCE', replay['reasons'])
        receiver.receive(message(3), 10.03, 100031000000, 1)
        self.assertTrue(receiver.snapshot(10.04, 100041000000)['cache_usable'])

    # 新正常包到来前先记录上次包的接收超时，避免健康尾样本抹掉空窗。
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

    # 即使空窗期间未调用snapshot，也应先撤销源样本过期缓存，再接受健康尾包。
    def test_source_gap_is_recorded_before_a_healthy_tail_without_snapshot(self):
        receiver = self.receiver(timeout_sec=1., max_speed_mps=.5, max_source_age_sec=.05,
                                 future_tolerance_sec=.01, expected_source_id='synthetic-session-1')
        receiver.receive(message(stamp=NS(sec=100, nanosec=0)), 10., 100000000000, 1)
        receiver.drain_events()
        accepted = receiver.receive(message(2, stamp=NS(sec=100, nanosec=100000000)),
                                    10.1, 100100000000, 1)
        self.assertTrue(accepted['accepted'])
        state = receiver.snapshot(10.1, 100100000000)
        self.assertTrue(state['cache_usable'])
        self.assertEqual(state['counters']['timeout'], 0)
        self.assertEqual(state['counters']['source_expired'], 1)
        self.assertEqual(state['last_anomaly']['kind'], 'cache_revoked')
        self.assertEqual(state['last_anomaly']['last_candidate_sequence'], 1)
        self.assertEqual(state['last_anomaly']['reasons'], ['SOURCE_AGE_LIMIT_EXCEEDED'])
        events = receiver.drain_events()
        self.assertEqual([value['kind'] for value in events], ['cache_revoked', 'message'])
        self.assertEqual(events[0]['event_id'], state['last_anomaly']['event_id'])

    # 源过期后紧跟invalid，须分别记录cache_revoked/message；同次过期不得重复计数。
    def test_source_gap_survives_invalid_tail_and_is_not_double_counted(self):
        receiver = self.receiver(timeout_sec=1., max_speed_mps=.5, max_source_age_sec=.05,
                                 future_tolerance_sec=.01, expected_source_id='synthetic-session-1')
        receiver.receive(message(stamp=NS(sec=100, nanosec=0)), 10., 100000000000, 1)
        receiver.drain_events()
        rejected = receiver.receive(message(2, valid=False, stamp=NS(sec=100, nanosec=100000000)),
                                    10.1, 100100000000, 1)
        self.assertFalse(rejected['accepted'])
        state = receiver.snapshot(10.11, 100110000000)
        self.assertFalse(state['cache_usable'])
        self.assertEqual(state['counters']['source_expired'], 1)
        events = receiver.drain_events()
        self.assertEqual([value['kind'] for value in events], ['cache_revoked', 'message'])
        self.assertIn('SOURCE_AGE_LIMIT_EXCEEDED', events[0]['reasons'])
        self.assertIn('COMMAND_INVALID', events[1]['reasons'])
        receiver.receive(message(3, stamp=NS(sec=100, nanosec=120000000)),
                         10.12, 100120000000, 1)
        state = receiver.snapshot(10.13, 100130000000)
        self.assertTrue(state['cache_usable'])
        self.assertEqual(state['counters']['source_expired'], 1)
        self.assertIn('COMMAND_INVALID', state['last_anomaly']['reasons'])

    # 历史容量满时拒绝新复位周期而不逐出旧证据，旧重放仍拒绝，当前已知周期仍可前进。
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

    # 40个携NaN的复位身份不能占用可信历史容量，后续正常新周期仍可接受。
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

    # 复位可重启序号，不能使确认过的源测量时钟倒退或不前进；新戳才可恢复。
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

    # 结构有效的撤销包同样推进跨复位测量高水位，下一周期不能复用撤销时戳。
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


# 文件证据边界：所有写入使用临时目录，检查轮转、节流、深复制和停机强制刷盘。
class WriterTests(unittest.TestCase):
    # 未指定输出目录时Writer不创建文件，保持只读默认。
    def test_default_writer_creates_nothing(self):
        writer = diagnostic.DiagnosticWriter()
        self.assertFalse(writer.record(dict(kind='message'), 1.))
        writer.snapshot(dict(control_authority=False))
        self.assertIsNone(writer.directory)

    # 100条记录触发轮转，主日志及一份备份均≤1024字节，快照将NaN写为null。
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

    # 节流期内唯一invalid不能被健康尾包覆盖；到期flush保留异常并记录合并丢写数。
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

    # 待刷盘异常须深复制调用者事件，外部嵌套修改不能伪造最终落盘载荷。
    def test_pending_anomaly_is_independent_from_callers_nested_event(self):
        with tempfile.TemporaryDirectory(prefix='fc01_velocity_diag_pending_copy_') as directory:
            writer = diagnostic.DiagnosticWriter(directory, log_min_interval_sec=60.)
            writer.record(dict(kind='message', event_id=1, reasons=[]), 10.)
            event = dict(kind='message', event_id=2, reasons=['COMMAND_INVALID'],
                         sample=dict(vx=.2, stamp=dict(sec=100, nanosec=1000000)))
            self.assertFalse(writer.record(event, 10.1))
            event['reasons'].clear()
            event['sample']['vx'] = 99.
            event['sample']['stamp']['sec'] = 999

            self.assertTrue(writer.flush(10.2, force=True))
            path = Path(directory) / 'velocity_command_diagnostic.log'
            saved = json.loads(path.read_text().splitlines()[-1])
            self.assertEqual(saved['event_id'], 2)
            self.assertEqual(saved['reasons'], ['COMMAND_INVALID'])
            self.assertEqual(saved['sample']['vx'], .2)
            self.assertEqual(saved['sample']['stamp']['sec'], 100)
            self.assertEqual(saved['coalesced_anomalies'], 1)
            self.assertEqual(saved['log_events_suppressed'], 1)
            self.assertFalse(writer.flush(10.2, force=True))

    # 停机强制flush可越过节流间隔，但同一异常只能写一次。
    def test_forced_flush_writes_pending_anomaly_once_before_interval(self):
        with tempfile.TemporaryDirectory(prefix='fc01_velocity_diag_force_') as directory:
            writer = diagnostic.DiagnosticWriter(directory, log_min_interval_sec=60.)
            writer.record(dict(kind='message', event_id=1, reasons=[]), 10.)
            writer.record(dict(kind='message', event_id=2, reasons=['COMMAND_INVALID']), 10.1)
            self.assertFalse(writer.flush(10.2))
            self.assertTrue(writer.flush(10.2, force=True))
            self.assertFalse(writer.flush(10.2, force=True))
            lines = [json.loads(line) for line in (Path(directory) / 'velocity_command_diagnostic.log').read_text().splitlines()]
            self.assertEqual(len(lines), 2)
            self.assertEqual(lines[-1]['event_id'], 2)
            self.assertEqual(lines[-1]['reasons'], ['COMMAND_INVALID'])

    # 模拟满盘导致强刷失败，待写异常必须保留，恢复IO后可重试一次。
    def test_failed_forced_flush_retains_pending_anomaly_for_retry(self):
        with tempfile.TemporaryDirectory(prefix='fc01_velocity_diag_force_retry_') as directory:
            writer = diagnostic.DiagnosticWriter(directory, log_min_interval_sec=60.)
            writer.record(dict(kind='message', event_id=1, reasons=[]), 10.)
            writer.record(dict(kind='message', event_id=2, reasons=['COMMAND_INVALID']), 10.1)
            with patch.object(writer, '_write', side_effect=OSError('synthetic full disk')):
                with self.assertRaises(OSError):
                    writer.flush(10.2, force=True)
            self.assertTrue(writer.flush(10.3, force=True))
            self.assertFalse(writer.flush(10.3, force=True))

    # 无需ROS即可finalize：强刷invalid、写停止快照、撤销候选，并保留分开的ROS/系统停止时钟。
    def test_shutdown_persists_pending_anomaly_and_terminal_snapshot_without_ros(self):
        with tempfile.TemporaryDirectory(prefix='fc01_velocity_diag_stop_') as directory:
            receiver = diagnostic.Receiver()
            writer = diagnostic.DiagnosticWriter(directory, log_min_interval_sec=60.)
            receiver.receive(message(valid=False), 10., 100001000000, 1)
            for event in receiver.drain_events():
                writer.record(event, 10.)
            value = diagnostic.finalize_diagnostic(receiver, writer, '/synthetic/velocity',
                                                  10.1, 100101000000)
            self.assertFalse(value['receiver_running'])
            self.assertFalse(value['cache_usable'])
            self.assertIsNone(value['current_candidate'])
            self.assertIn('RECEIVER_STOPPED', value['cache_errors'])
            self.assertEqual(value['publisher_count'], 1)
            self.assertEqual(value['stopped_at_monotonic'], 10.1)
            self.assertEqual(value['stopped_at_ros_ns'], 100001000000)
            self.assertEqual(value['stopped_at_system_ns'], 100101000000)
            self.assertEqual(value['last_message']['sequence'], 1)
            snapshot = json.loads((Path(directory) / 'velocity_command_diagnostic.json').read_text())
            self.assertEqual(snapshot, value)
            self.assertIn('COMMAND_INVALID', snapshot['latest_errors'])
            lines = [json.loads(line) for line in (Path(directory) / 'velocity_command_diagnostic.log').read_text().splitlines()]
            self.assertTrue(any('COMMAND_INVALID' in row['reasons'] for row in lines))

    # 系统纪元与仿真ROS相差巨大时，停机用最后观测ROS时间，不能伪造过期或时钟回退。
    def test_shutdown_does_not_substitute_system_time_for_simulated_ros_clock(self):
        receiver = diagnostic.Receiver(diagnostic.DiagnosticConfig(
            timeout_sec=1., max_speed_mps=.5, max_source_age_sec=.05,
            future_tolerance_sec=.01, expected_source_id='synthetic-session-1'))
        receiver.receive(message(stamp=NS(sec=100, nanosec=0)), 10., 100000000000, 1)
        receiver.drain_events()
        system_now_ns = 1700000000000000000
        value = diagnostic.finalize_diagnostic(receiver, diagnostic.DiagnosticWriter(),
                                              '/synthetic/velocity', 10.01, system_now_ns)
        self.assertEqual(value['stopped_at_ros_ns'], 100000000000)
        self.assertEqual(value['stopped_at_system_ns'], system_now_ns)
        self.assertEqual(value['shutdown_clock_source'], 'last_observed_ros_clock')
        self.assertEqual(value['counters']['source_expired'], 0)
        self.assertEqual(value['counters']['clock_regression'], 0)
        self.assertIsNone(value['last_anomaly'])
        self.assertFalse(value['cache_usable'])
        self.assertIsNone(value['current_candidate'])
        self.assertIn('RECEIVER_STOPPED', value['cache_errors'])

    # 从未观测ROS时钟时保持0未知，不用系统时间冒充。
    def test_shutdown_without_ros_observation_keeps_clock_unknown(self):
        value = diagnostic.finalize_diagnostic(diagnostic.Receiver(), diagnostic.DiagnosticWriter(),
                                              '/synthetic/velocity', 10., 1700000000000000000)
        self.assertEqual(value['stopped_at_ros_ns'], 0)
        self.assertEqual(value['shutdown_clock_source'], 'last_observed_ros_clock')
        self.assertFalse(value['receiver_running'])

    # 日志record/flush失败后仍尝试写停止快照，并向调用者保留最先的IO异常。
    def test_shutdown_snapshot_is_attempted_after_log_io_failure(self):
        receiver = diagnostic.Receiver()
        receiver.receive(message(valid=False), 10., 100001000000, 1)
        writer = diagnostic.DiagnosticWriter()
        with patch.object(writer, 'record', side_effect=OSError('synthetic log failure')), \
             patch.object(writer, 'flush', side_effect=OSError('synthetic flush failure')), \
             patch.object(writer, 'snapshot') as snapshot:
            with self.assertRaisesRegex(OSError, 'synthetic log failure'):
                diagnostic.finalize_diagnostic(receiver, writer, '/synthetic/velocity', 10.1, 100101000000)
            snapshot.assert_called_once()
            self.assertFalse(snapshot.call_args.args[0]['receiver_running'])

    # 单条10000字符事件须替换为LOG_RECORD_TOO_LARGE证据，不能突破文件上限。
    def test_oversize_event_cannot_break_file_limit(self):
        with tempfile.TemporaryDirectory(prefix='fc01_velocity_diag_oversize_') as directory:
            writer = diagnostic.DiagnosticWriter(directory, log_max_bytes=1024)
            writer.record(dict(kind='message', event_id=1, payload='z' * 10000), 10.)
            path = Path(directory) / 'velocity_command_diagnostic.log'
            self.assertLessEqual(path.stat().st_size, 1024)
            self.assertEqual(json.loads(path.read_text())['reasons'], ['LOG_RECORD_TOO_LARGE'])

    # 静态检查独立工具不含硬件发布/导航命令入口，ROS依赖只在main内加载，纯逻辑可离线导入。
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
