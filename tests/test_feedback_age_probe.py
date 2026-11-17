# 反馈年龄探针的纯诊断算术回归，分别保留消息封套、原始样本及 RMW 接收时间。
# 不修正飞控源时间，不把趋势或收到新封套解释为控制有效性。
"""Diagnostic arithmetic only; no ROS/Agent/serial needed."""
import argparse
import json
from pathlib import Path
import tempfile
import unittest

from feedback_age_probe import (
    StampProgress,
    TopicObservation,
    age_seconds,
    duration_arg,
    number,
    read_controller_status,
    slope,
)


# 验证时间缺失、未来值、冻结样本、时钟跳变及趋势窗口的显示证据。
class FeedbackAgeProbeTests(unittest.TestCase):
    # 合法 JSON 允许较大的整数；float 检查溢出应与 NaN/Inf 一样保持未知，而非终止探针。
    def test_number_rejects_both_signs_of_integer_overflow(self):
        for value in (10**400, -(10**400)):
            with self.subTest(positive=value > 0):
                self.assertIsNone(number(value))

    # 保留原标量与类型，不以新容差或零替代合法值和未知值。
    def test_number_keeps_finite_values_and_original_type(self):
        for value in (0, -2, 2**64 - 1, 1.25, -0.0):
            with self.subTest(value=value):
                self.assertEqual(number(value), value)
                self.assertIs(type(number(value)), type(value))
        for value in (True, None, '12', float('nan'), float('inf'), -float('inf')):
            self.assertIsNone(number(value))

    # 文件只读且 JSON 语法完全有效；异常大数只让对应诊断值未知，不篡改原文件。
    def test_status_snapshot_large_numeric_diagnostics_remain_available(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'status.json'
            fixture = dict(
                updated_unix=10**400,
                state_age_seconds=-(10**400),
                state=dict(diagnostic_host_us=10**400, diagnostic_monotonic_s=10**400),
            )
            path.write_text(json.dumps(fixture))
            before = path.read_bytes()
            row = read_controller_status(path, 1_000_000, {})
            self.assertTrue(row['available'])
            for name in (
                'updated_unix',
                'state_age_seconds',
                'diagnostic_host_us',
                'diagnostic_monotonic_s',
                'status_wall_age_s',
            ):
                self.assertIsNone(row[name])
            self.assertEqual(path.read_bytes(), before)

    # 本地快照不受 ROS uint64 字段约束；源戳算术溢出沿原错误报告路径返回，文件保持只读。
    def test_status_snapshot_stamp_overflow_returns_explicit_partial_diagnostic(self):
        for envelope in (10**400, -(10**400)):
            for sample in (10**400, -(10**400)):
                with (
                    self.subTest(envelope_positive=envelope > 0, sample_positive=sample > 0),
                    tempfile.TemporaryDirectory() as directory,
                ):
                    path = Path(directory) / 'status.json'
                    fixture = dict(
                        updated_unix=1.0,
                        state=dict(
                            source_timing=dict(local=dict(envelope_us=envelope, sample_us=sample))
                        ),
                    )
                    path.write_text(json.dumps(fixture))
                    before = path.read_bytes()
                    row = read_controller_status(
                        path, 1_000_000, {'local': {'last': {'envelope_us': 100}}}
                    )
                    # available 仍只表示已解析根对象；error 明确保存算术失败，不代表反馈有效。
                    self.assertTrue(row['available'])
                    self.assertEqual(row['error'], 'int too large to convert to float')
                    self.assertEqual(row['topics'], {})
                    self.assertEqual(path.read_bytes(), before)

    # 0/缺失时间必须保持未知，不能自动换成主机当前时刻。
    def test_missing_timestamp_and_rmw_zero_are_unavailable(self):
        observed = TopicObservation()
        observed.observe(0, 0, {'received_timestamp': 0}, 1_000_000, 10.0)
        row = observed.report(1_000_010, 10.01, 5.0)
        self.assertIsNone(row['last']['callback_envelope_age_s'])
        self.assertIsNone(row['last']['callback_sample_age_s'])
        self.assertIsNone(row['last']['rmw_to_callback_s'])
        self.assertEqual(row['last']['rmw_received_timestamp_ns'], 0)
        self.assertFalse(row['last']['rmw_received_available'])

    # 保留负年龄揭示未来源时间或时钟偏差，不能裁成零掩盖同步问题。
    def test_signed_age_and_rmw_gap_preserve_future_values(self):
        observed = TopicObservation()
        observed.observe(
            1_100_000, 1_050_000, {'received_timestamp': 1_010_000_000}, 1_000_000, 10.0
        )
        self.assertAlmostEqual(observed.last['callback_envelope_age_s'], -0.1)
        self.assertAlmostEqual(observed.last['rmw_to_callback_s'], -0.01)

    # 外层时间更新但 sample 不变时，样本年龄继续增加并记为 duplicate。
    def test_new_envelope_cannot_refresh_frozen_old_raw(self):
        observed = TopicObservation()
        observed.observe(10_000_000, 9_400_000, {}, 10_000_000, 10.0)
        observed.observe(10_050_000, 9_400_000, {}, 10_050_000, 10.05)
        self.assertEqual(observed.last['callback_envelope_age_s'], 0.0)
        self.assertAlmostEqual(observed.last['callback_sample_age_s'], 0.65)
        self.assertEqual(observed.sample_progress.counts['duplicate'], 1)

    # 分别计数首次、前进、重复、倒退和缺失；这些事件不能混为单一丢包率。
    def test_sample_progress_records_duplicate_regression_and_missing(self):
        progress = StampProgress()
        self.assertEqual(
            [progress.observe(x) for x in (10, 20, 20, 15, 0)],
            ['first', 'advanced', 'duplicate', 'regressed', 'missing'],
        )
        self.assertEqual(
            progress.counts, dict(first=1, advanced=1, duplicate=1, regressed=1, missing=1)
        )

    # 构造每秒增加 0.002 秒的年龄趋势；窗口过期后应清空拟合结果。
    def test_fixed_window_slope_and_expiry(self):
        observed = TopicObservation()
        for mono in (1.0, 2.0, 3.0):
            host = 1_000_000_000 + int(mono * 1e6)
            envelope = host - int((0.1 + 0.002 * mono) * 1e6)
            observed.observe(envelope, envelope - 2000, {}, host, mono)
        row = observed.report(host, 3.0, 3.0)
        trend = row['envelope_age_trend']
        self.assertAlmostEqual(trend['slope_s_per_s'], 0.002)
        self.assertAlmostEqual(trend['r_squared'], 1.0)
        expired = observed.report(host, 34.0, 31.0)
        self.assertEqual(expired['envelope_age_trend']['samples'], 0)
        self.assertIsNone(expired['envelope_age_trend']['slope_s_per_s'])

    # 墙钟跳变应显露在诊断年龄中，原始源戳必须原样保留。
    def test_wall_jump_changes_diagnostics_instead_of_correcting_source(self):
        observed = TopicObservation()
        observed.observe(1_000_000, 999_000, {}, 1_000_000, 1.0)
        observed.observe(1_050_000, 1_049_000, {}, 2_050_000, 1.05)
        self.assertAlmostEqual(observed.last['callback_envelope_age_s'], 1.0)
        self.assertEqual(observed.last['envelope_us'], 1_050_000)
        self.assertLess(age_seconds(1_000_000, 2_000_000), 0.0)

    # 只读载入控制器快照并区分其观测时刻；不把不同采样时点的年龄强行比较为相同。
    def test_read_only_controller_comparison_distinguishes_sample_instants(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'status.json'
            fixture = dict(
                updated_unix=10.0,
                feedback_current=False,
                state_controller_pid=23,
                state=dict(
                    diagnostic_host_us=10_000_000,
                    source_timing=dict(
                        local=dict(envelope_us=9_800_000, sample_us=9_799_000, envelope_age_s=0.2)
                    ),
                ),
            )
            path.write_text(json.dumps(fixture))
            row = read_controller_status(
                path, 10_100_000, {'local': {'last': {'envelope_us': 10_000_000}}}
            )
            self.assertTrue(row['available'])
            self.assertFalse(row['feedback_current'])
            self.assertFalse(row['topics']['local']['same_envelope'])
            self.assertAlmostEqual(row['topics']['local']['probe_minus_controller_envelope_s'], 0.2)
            self.assertEqual(json.loads(path.read_text()), fixture)

    # CLI 采集时长须有限且处于允许区间，防止意外无界采集。
    def test_duration_is_bounded(self):
        self.assertEqual(duration_arg('5'), 5.0)
        self.assertEqual(duration_arg('180'), 180.0)
        for value in ('4', '181', 'nan', 'inf'):
            with self.assertRaises(argparse.ArgumentTypeError):
                duration_arg(value)

    # 斜率拟合需要至少两个不同的接收时刻，不能在退化时间轴上产生伪结果。
    def test_slope_needs_multiple_different_receipt_instants(self):
        self.assertIsNone(slope([])['slope_s_per_s'])
        self.assertIsNone(slope([(1.0, 0.2)])['slope_s_per_s'])
        self.assertIsNone(slope([(1.0, 0.2), (1.0, 0.3)])['slope_s_per_s'])


if __name__ == '__main__':
    unittest.main()
