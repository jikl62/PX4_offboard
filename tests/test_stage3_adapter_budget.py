"""适配器入口预算回归：覆盖直接转换、未配对缓存、异常撤销及正常 UTF-8 边界。"""

from dataclasses import replace
import unittest

import test_vision_adapter as fixture


class AdapterResourceBudgetTests(unittest.TestCase):
    """使用真实 GuardedOdometryAdapter；输入是 ROS 形状假消息，不创建通信节点。"""

    @classmethod
    def setUpClass(cls):
        """沿用已审查的坐标/标定合成夹具，不复制转换算法。"""
        fixture.OdometryAdapterTests.setUpClass()

    def setUp(self):
        """每例重建全部缓存与配置，避免先前异常影响后续案例。"""
        helper = fixture.OdometryAdapterTests()
        helper.setUp()
        self.adapter, self.config = helper.a, helper.config

    def test_configuration_checks_bytes_before_identity_content(self):
        """必填身份仍非空；超预算/NUL/非法 Unicode 作为明确新合同错误。"""
        for field in (
            'source_id',
            'input_frame_id',
            'input_body_frame_id',
            'extrinsics_id',
            'alignment_id',
        ):
            for good in ('a' * 256, 'é' * 128):
                replace(self.config, **{field: good}).validate()
            for bad in ('a' * 257, 'é' * 129, 'a\0b', '\ud800'):
                with self.subTest(field=field, value=repr(bad[:3])):
                    with self.assertRaisesRegex(ValueError, 'VISION_IDENTITY_RESOURCE_LIMIT'):
                        replace(self.config, **{field: bad}).validate()

    def test_every_raw_identity_is_rejected_without_converted_copy(self):
        """逐个检查世界/机体/来源/标定/角速度身份，输出不携带异常原文。"""
        fields = [('raw', 'frame_id'), ('raw', 'child_frame_id')] + [
            ('health', field)
            for field in (
                'source_id',
                'frame_id',
                'body_frame_id',
                'angular_velocity_source',
                'extrinsics_id',
                'alignment_id',
            )
        ]
        for owner, field in fields:
            with self.subTest(owner=owner, field=field):
                raw, health = fixture.odometry(), fixture.health()
                target = (
                    raw.header
                    if owner == 'raw' and field == 'frame_id'
                    else raw if owner == 'raw' else health
                )
                setattr(target, field, 'x' * (1024 * 1024))
                converted = self.adapter.convert_odometry(raw, health, self.config)
                self.assertEqual(converted.valid_fields, 0)
                self.assertEqual(converted.reason, 'VISION_IDENTITY_RESOURCE_LIMIT')
                self.assertEqual(converted.source_id, self.config.source_id)
                self.assertEqual(converted.angular_velocity_source, '')

    def test_invalid_half_packet_revokes_and_clears_both_pending_sides(self):
        """即使坏包时间很旧，也撤销当前资格并清半包；不借用旧健康数据重新配对。"""
        for method in ('odometry', 'health'):
            with self.subTest(method=method):
                guarded = self.adapter.GuardedOdometryAdapter(self.config)
                guarded.odometry(fixture.odometry(), 1, 10000000025, 1, 1)
                self.assertEqual(len(guarded.pairs._odometry), 1)
                bad = fixture.odometry() if method == 'odometry' else fixture.health()
                if method == 'odometry':
                    bad.header.frame_id = 'x' * (1024 * 1024)
                    bad.header.stamp.sec = 1
                else:
                    bad.source_id = 'x' * (1024 * 1024)
                    bad.stamp.sec = 1
                states = getattr(guarded, method)(bad, 1.01, 10010000025, 1, 1)
                self.assertEqual([s.reason for s in states], ['VISION_IDENTITY_RESOURCE_LIMIT'])
                self.assertTrue(all(s.valid_fields == 0 for s in states))
                self.assertFalse(guarded.pairs._odometry)
                self.assertFalse(guarded.pairs._health)
                self.assertEqual(guarded.health(fixture.health(), 1.02, 10020000025, 1, 1), [])

    def test_expiry_precedes_new_resource_rejection(self):
        """超时和本次资源拒绝均保留为独立事件，后者不能覆盖先前超时。"""
        guarded = self.adapter.GuardedOdometryAdapter(self.config)
        guarded.odometry(fixture.odometry(), 1, 10000000025, 1, 1)
        bad = fixture.health(source_id='x' * 257)
        states = guarded.health(bad, 1.06, 10060000025, 1, 1)
        self.assertEqual(
            [s.reason for s in states], ['MISSING_HEALTH', 'VISION_IDENTITY_RESOURCE_LIMIT']
        )
        self.assertEqual([s.sequence for s in states], [1, 2])
        self.assertFalse(guarded.pairs._odometry)
        self.assertFalse(guarded.pairs._health)

    def test_standalone_pair_buffer_never_retains_oversize_half_packet(self):
        """低层配对器不负责发布，但独立使用时仍显式拒绝大身份，不创建缓存项。"""
        for method in ('odometry', 'health'):
            pairs = self.adapter.StampPairBuffer()
            bad = fixture.odometry() if method == 'odometry' else fixture.health()
            if method == 'odometry':
                bad.child_frame_id = 'x' * 257
            else:
                bad.source_id = 'x' * 257
            with self.assertRaisesRegex(ValueError, 'VISION_IDENTITY_RESOURCE_LIMIT'):
                getattr(pairs, method)(bad, 1)
            self.assertFalse(pairs._odometry)
            self.assertFalse(pairs._health)

    def test_valid_boundary_then_invalid_packet_then_fresh_pair(self):
        """256 UTF-8 字节身份能正常转换；拒绝后新配对可恢复，不截断身份冒充正常。"""
        config = replace(self.config, source_id='é' * 128)
        guarded = self.adapter.GuardedOdometryAdapter(config)

        def pair(now):
            raw, health = fixture.odometry(), fixture.health(source_id=config.source_id)
            raw.header.stamp.nanosec = health.stamp.nanosec = int(now * 1000000)
            guarded.odometry(raw, now, 10000000000 + int(now * 1000000), 1, 1)
            return guarded.health(health, now, 10000000000 + int(now * 1000000), 1, 1)

        self.assertEqual(pair(1)[0].valid_fields & 71, 71)
        rejected = guarded.health(fixture.health(source_id='é' * 129), 1.01, 10010000000, 1, 1)
        self.assertEqual(rejected[0].valid_fields, 0)
        self.assertEqual(pair(1.02)[0].valid_fields & 71, 71)
