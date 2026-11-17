"""应用层视觉身份预算：真实校验器/接收器、字节边界、撤销和有界诊断，不访问 ROS 或硬件。"""

from copy import deepcopy
import importlib.util
import json
from pathlib import Path
import re
import sys
import unittest

import test_velocity_command_diagnostic as velocity_fixture
import test_vision_state_validation as state_fixture


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    'fc01_stage3_budget', ROOT / 'native/tools/vision_resource_budget.py'
)
budget = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = budget
SPEC.loader.exec_module(budget)


class VisionResourceBudgetTests(unittest.TestCase):
    """256 字节来自既有离线预览合同；保留 V1 消息身份，不截断后授予资格。"""

    def test_cpp_and_python_share_utf8_byte_budget(self):
        source = (
            ROOT / 'native/src/jikl_fc01_system/include/jikl_fc01_system/vision_resource_budget.hpp'
        ).read_text()
        cpp_limit = int(re.search(r'identity_max_bytes\s*=\s*(\d+)', source)[1])
        self.assertEqual(cpp_limit, budget.IDENTITY_MAX_BYTES)
        self.assertEqual(velocity_fixture.diagnostic.MAX_TEXT, cpp_limit)
        for value in ('a' * 256, 'é' * 128, '🚁' * 64):
            with self.subTest(bytes=len(value.encode('utf-8'))):
                self.assertTrue(budget.identity_within_budget(value))
                self.assertFalse(budget.identity_within_budget(value + 'a'))
        for value in ('a' * (1024 * 1024), 'a\0b', '\ud800', 23, None):
            with self.subTest(type=type(value).__name__):
                self.assertFalse(budget.identity_within_budget(value))
        self.assertTrue(budget.identity_within_budget(''))
        self.assertFalse(budget.valid_identity(''))
        self.assertTrue(budget.valid_identity('', allow_empty=True))

    def test_log_truncation_uses_bytes_and_does_not_validate_truncated_identity(self):
        value = '🚁' * (1024 * 1024)
        displayed = budget.bounded_text(value)
        self.assertEqual(len(displayed.encode('utf-8')), 256)
        self.assertFalse(budget.valid_identity(value))
        self.assertIsNone(budget.bounded_text(None))
        self.assertLessEqual(len(budget.bounded_text('\ud800' * 256).encode('utf-8')), 256)

    def test_velocity_exact_unicode_limit_and_oversize_revoke_without_history_growth(self):
        module = velocity_fixture.diagnostic
        identity = 'é' * 128
        receiver = module.Receiver(module.DiagnosticConfig(expected_source_id=identity))
        accepted = receiver.receive(
            velocity_fixture.message(source_id=identity), 10, 100001000000, 1
        )
        self.assertTrue(accepted['accepted'])
        previous = deepcopy(receiver._watermarks)
        invalid = receiver.receive(
            velocity_fixture.message(sequence=0, source_id='🚁' * 1000000),
            10.01,
            100011000000,
            1,
        )
        self.assertFalse(invalid['accepted'])
        self.assertIn('SOURCE_ID_INVALID', invalid['reasons'])
        self.assertIsNone(receiver._candidate)
        self.assertEqual(receiver._watermarks, previous)
        self.assertLessEqual(len(invalid['sample']['source_id'].encode('utf-8')), 256)
        self.assertLess(len(json.dumps(invalid, ensure_ascii=False).encode('utf-8')), 4096)
        recovered = receiver.receive(
            velocity_fixture.message(sequence=2, source_id=identity), 10.02, 100021000000, 1
        )
        self.assertTrue(recovered['accepted'])
        self.assertIsNotNone(receiver.last_anomaly)
        for bad in (identity + 'é', 'a\0b', '\ud800'):
            with self.subTest(identity=repr(bad[:3])):
                with self.assertRaises(ValueError):
                    module.DiagnosticConfig(expected_source_id=bad)

    def test_every_raw_identity_is_rejected_before_cache_or_diagnostics(self):
        module = state_fixture.m
        # 即使异常消息序号/源戳都落后，资源拒绝仍必须撤销当前候选，不能忽略后续事件。
        cases = [('raw', 'frame_id'), ('raw', 'child_frame_id')] + [
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
        for owner, field in cases:
            with self.subTest(owner=owner, field=field):
                validator = module.Validator(state_fixture.config())
                raw, health = state_fixture.pair()
                self.assertTrue(validator.receive(raw, health, 1, 10050000000).accepted)
                trusted = deepcopy(validator._trusted_epochs)
                bad_raw, bad_health = state_fixture.pair(sequence=0, stamp_ns=1)
                target = (
                    bad_raw.header
                    if owner == 'raw' and field == 'frame_id'
                    else bad_raw if owner == 'raw' else bad_health
                )
                setattr(target, field, 'x' * (1024 * 1024))
                rejected = validator.receive(bad_raw, bad_health, 1.01, 10060000000)
                self.assertFalse(rejected.accepted)
                self.assertIn(budget.RESOURCE_ERROR, rejected.diagnostics['reasons'])
                self.assertIsNone(validator._cache)
                self.assertEqual(validator._trusted_epochs, trusted)
                self.assertLess(
                    len(json.dumps(rejected.diagnostics, ensure_ascii=False).encode('utf-8')),
                    8192,
                )
                raw, health = state_fixture.pair(sequence=2)
                self.assertTrue(validator.receive(raw, health, 1.05, 10100000000).accepted)

    def test_state_config_and_metadata_budget_are_bytes(self):
        module = state_fixture.m
        # 无需真实标定数据也可检查标识边界；数值/坐标协议仍使用原显式合成配置。
        for field in (
            'source_id',
            'input_frame_id',
            'input_body_frame_id',
            'extrinsics_id',
            'alignment_id',
        ):
            with self.subTest(field=field):
                state_fixture.config(adapter=state_fixture.adapter_config(**{field: 'é' * 128}))
                for bad in ('é' * 129, 'a\0b', '\ud800'):
                    with self.assertRaises(ValueError):
                        state_fixture.config(adapter=state_fixture.adapter_config(**{field: bad}))
        module.SampleMetadata(relocalization_id='🚁' * 64)
        for bad in ('🚁' * 65, 'a\0b', '\ud800'):
            with self.assertRaises(ValueError):
                module.SampleMetadata(relocalization_id=bad)


if __name__ == '__main__':
    unittest.main()
