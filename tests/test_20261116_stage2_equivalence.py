"""阶段 2：对 3677b9d 基线严格对照控制，只允许新增只读诊断字段。

复用既有真实 Core、Runtime、SystemNode 和发布 sink 夹具，不复制控制算法。
节点的既有 JSON 成员保留原字节，所有目标、心跳、命令字段和顺序直接比较；
只有本轮明确列出的新增顶层诊断键可以从当前输出中剔除，不能忽略旧字段差异。
"""

import hashlib
import json
import os
from pathlib import Path
import subprocess
import unittest
from unittest.mock import patch

import test_stage2_powered_node_equivalence as powered_trace
import test_stage3_control_equivalence as stage3_trace


BASELINE = '3677b9dd1f35e092e43501d26164c6263545c947'
EVIDENCE_ENV = 'FC01_20261116_STAGE2_TRACE_DIR'
# 明确枚举本轮新增字段，避免未来同前缀的控制字段也被无意跳过。
ADDITIVE_DIAGNOSTIC_KEYS = frozenset(
    (
        'range_policy',
        'range_available',
        'range_compared',
        'range_distance_m',
        'range_variance_m2',
        'range_fused_rise_m',
        'range_rise_m',
        'range_interval_s',
        'range_same_direction',
        'range_sample_stamp',
        'rc_queued_safety_recorded',
        'rc_queued_safety_mask',
        'rc_queued_safety_source',
        'rc_queued_safety_observed_at',
        'rc_queued_safety_status_stamp',
        'rc_queued_safety_land_stamp',
        'rc_queued_safety_local_stamp',
        'rc_queued_safety_switch_stamp',
    )
)


def _source_identity():
    """记录真实编译输入；同一轮中源码有变更就拒绝给出跨版本的通过证据。"""
    root = Path(__file__).resolve().parents[1]
    files = {
        path
        for folder in (root / 'native/src/jikl_fc01_system', root / 'native/tests')
        for path in folder.rglob('*')
        if path.is_file() and path.suffix in ('.cpp', '.hpp', '.py')
    }
    files.update(
        root / 'tests' / name
        for name in (
            'test_20261116_stage2_equivalence.py',
            'test_stage2_core_equivalence.py',
            'test_stage2_powered_node_equivalence.py',
            'test_stage3_control_equivalence.py',
        )
    )
    return {
        path.relative_to(root).as_posix(): hashlib.sha256(path.read_bytes()).hexdigest()
        for path in sorted(files)
    }


def setUpModule():
    """所有差分场景开始前固定本轮源码摘要，不切换分支或改写源码。"""
    global _SOURCE_BEFORE
    _SOURCE_BEFORE = _source_identity()


def tearDownModule():
    """测试完成后再次核对源码，防止并行编辑让不同场景使用不同版本。"""
    after = _source_identity()
    root = Path(__file__).resolve().parents[1]
    evidence = Path(os.environ.get(EVIDENCE_ENV, '/tmp/fc01-stage2-differential'))
    evidence.mkdir(parents=True, exist_ok=True)
    changed = sorted(
        key for key in _SOURCE_BEFORE.keys() | after.keys() if _SOURCE_BEFORE.get(key) != after.get(key)
    )
    summary = {
        'baseline': BASELINE,
        'current_head': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=root, text=True).strip(),
        'source_unchanged_during_run': not changed,
        'changed_during_run': changed,
        'source_sha256': after,
        'scope': 'Offline fake ROS and deterministic inputs; no DDS or hardware execution.',
    }
    (evidence / 'SOURCE_IDENTITY.json').write_text(json.dumps(summary, indent=2) + '\n')
    if changed:
        raise AssertionError('production or fixture sources changed during differential: ' + ', '.join(changed))


def _reject_constant(value):
    raise ValueError('non-standard JSON constant: ' + value)


def _unique_object(pairs):
    """拒绝重复键，不能让 JSON 解析器以后值覆盖前值而漏掉状态变化。"""
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError('duplicate JSON key: ' + key)
        result[key] = value
    return result


def _json_members(raw):
    """提取紧凑顶层对象的成员原文，数值字面量、嵌套值和成员顺序均不重写。"""
    text = raw.decode('utf-8')
    decoder = json.JSONDecoder(object_pairs_hook=_unique_object, parse_constant=_reject_constant)
    decoded, end = decoder.raw_decode(text)
    if not isinstance(decoded, dict) or end != len(text) or not text.startswith('{'):
        raise ValueError('expected one complete JSON object')
    if text == '{}':
        return []
    position = 1
    members = []
    while position < len(text) - 1:
        start = position
        key, position = decoder.raw_decode(text, position)
        if not isinstance(key, str) or text[position : position + 1] != ':':
            raise ValueError('expected compact JSON member')
        _, position = decoder.raw_decode(text, position + 1)
        members.append((key, text[start:position].encode('utf-8')))
        separator = text[position : position + 1]
        if separator == '}':
            if position != len(text) - 1:
                raise ValueError('trailing JSON data')
            return members
        if separator != ',':
            raise ValueError('expected compact JSON member separator')
        position += 1
    raise ValueError('missing JSON member')


def remove_new_diagnostics(current_trace, baseline_trace):
    """按相同位置的基线 JSON 仅去新增诊断成员；非状态行一字节也不处理。"""
    old_lines = baseline_trace.split(b'\n')
    new_lines = current_trace.split(b'\n')
    if len(old_lines) != len(new_lines):
        raise AssertionError('trace line count changed')
    filtered = []
    removed = set()
    states = 0
    for before, after in zip(old_lines, new_lines):
        if before.startswith((b'TRACE STATE ', b'POWERED STATE ')):
            before_prefix, before_json = before.split(b'{', 1)
            after_prefix, after_json = after.split(b'{', 1)
            if before_prefix != after_prefix:
                raise AssertionError('state sample identity changed')
            previous_members = _json_members(b'{' + before_json)
            current_members = _json_members(b'{' + after_json)
            previous_keys = {key for key, _ in previous_members}
            if previous_keys & ADDITIVE_DIAGNOSTIC_KEYS:
                raise AssertionError('allowed additive key already existed in baseline')
            retained = []
            for key, member in current_members:
                if key in ADDITIVE_DIAGNOSTIC_KEYS:
                    removed.add(key)
                else:
                    retained.append(member)
            filtered.append(after_prefix + b'{' + b','.join(retained) + b'}')
            states += 1
        else:
            # 包括 TARGET / HEARTBEAT / COMMAND / COMPLETE；调用方逐字节比较这些原行。
            filtered.append(after)
    return b'\n'.join(filtered), {'state_samples': states, 'removed_keys': sorted(removed)}


class DiagnosticFilterTests(unittest.TestCase):
    """过滤器必须只允许新增诊断，不能吞掉浮点、权限、顺序或消息差异。"""

    def test_only_additive_top_level_members_are_removed(self):
        baseline = b'TRACE STATE 0:{"height":1.00,"armed":false,"nested":{"range_policy":1}}'
        current = (
            b'TRACE STATE 0:{"height":1.00,"range_available":false,"armed":false,'
            b'"nested":{"range_policy":1},"rc_queued_safety_mask":0}'
        )
        filtered, audit = remove_new_diagnostics(current, baseline)
        self.assertEqual(filtered, baseline)
        self.assertEqual(audit['removed_keys'], ['range_available', 'rc_queued_safety_mask'])

    def test_existing_values_order_and_sink_bytes_remain_visible(self):
        baseline = b'POWERED STATE 0:{"height":1.00,"armed":false}\nPOWERED TARGET 0:0x1p-2'
        for current in (
            baseline.replace(b'1.00', b'1.0'),
            baseline.replace(b'"armed":false', b'"armed":true'),
            baseline.replace(b'"height":1.00,"armed":false', b'"armed":false,"height":1.00'),
            baseline.replace(b'0x1p-2', b'0x1p-1'),
            baseline.replace(b'"armed":false', b'"armed":false,"range_control_permission":true'),
        ):
            with self.subTest(current=current):
                filtered, _ = remove_new_diagnostics(current, baseline)
                self.assertNotEqual(filtered, baseline)

    def test_duplicate_keys_or_baseline_diagnostic_cannot_be_hidden(self):
        for before, after in (
            (b'{"range_available":false}', b'{"range_available":true}'),
            (b'{"height":1}', b'{"height":1,"height":2}'),
            (b'{"nested":{}}', b'{"nested":{"armed":false,"armed":true}}'),
        ):
            with self.subTest(after=after):
                with self.assertRaises((ValueError, AssertionError)):
                    remove_new_diagnostics(b'TRACE STATE 0:' + after, b'TRACE STATE 0:' + before)


class _DiagnosticComparisonMixin:
    """既有夹具没有过滤 hook；仅截获指定 trace 最终比较，其他断言维持原行为。"""

    _diagnostic_trace_name = None

    def _correct_inherited_summary(self, filename, comparison):
        """原夹具摘要默认声称全部字节相同；本轮必须明确写出只比较既有诊断成员。"""
        evidence = Path(os.environ.get(EVIDENCE_ENV, '/tmp/fc01-stage2-differential'))
        path = evidence / filename
        summary = json.loads(path.read_text())
        summary['comparison'] = 'exact-existing-json-members-and-wire-bytes'
        summary['allowed_additive_keys'] = sorted(ADDITIVE_DIAGNOSTIC_KEYS)
        summary['comparison_evidence'] = comparison
        summary['raw_json_byte_identity_claimed'] = False
        path.write_text(json.dumps(summary, ensure_ascii=False, indent=2) + '\n')

    def assertEqual(self, first, second, msg=None):
        messages = {
            '节点状态字节/发布顺序与基线不一致',
            'powered node behavior differs from 5198d33',
        }
        if self._diagnostic_trace_name is not None and msg in messages:
            if not isinstance(first, bytes) or not isinstance(second, bytes):
                raise AssertionError('trace comparison no longer supplies bytes')
            normalized, audit = remove_new_diagnostics(second, first)
            evidence = Path(os.environ.get(EVIDENCE_ENV, '/tmp/fc01-stage2-differential'))
            evidence.mkdir(parents=True, exist_ok=True)
            stem = self._diagnostic_trace_name
            (evidence / (stem + '-baseline.exact.log')).write_bytes(first)
            (evidence / (stem + '-current.filtered.log')).write_bytes(normalized)
            (evidence / (stem + '-current.raw.log')).write_bytes(second)
            audit.update(
                baseline=BASELINE,
                comparison='exact-existing-json-members-and-wire-bytes',
                existing_member_serialization='preserved raw JSON tokens and member order',
                baseline_sha256=hashlib.sha256(first).hexdigest(),
                current_raw_sha256=hashlib.sha256(second).hexdigest(),
                current_filtered_sha256=hashlib.sha256(normalized).hexdigest(),
                allowed_additive_keys=sorted(ADDITIVE_DIAGNOSTIC_KEYS),
                passed=first == normalized,
                hardware_or_dds_validation=False,
            )
            (evidence / (stem + '-comparison.json')).write_text(
                json.dumps(audit, ensure_ascii=False, indent=2) + '\n'
            )
            super().assertEqual(first, normalized, msg)
            return
        super().assertEqual(first, second, msg)


class Stage2CoreEquivalenceTests(_DiagnosticComparisonMixin, stage3_trace.Stage3CoreEquivalenceTests):
    """Core/事务/发布组合精确差分；节点只允许额外顶层诊断字段。"""

    @classmethod
    def setUpClass(cls):
        evidence = os.environ.get(EVIDENCE_ENV, '/tmp/fc01-stage2-differential')
        with patch.object(stage3_trace, 'BASELINE', BASELINE), patch.dict(
            os.environ, {'FC01_STAGE3_TRACE_DIR': evidence}
        ):
            super().setUpClass()

    def compare_cpp(self, stem):
        with patch.object(stage3_trace, 'BASELINE', BASELINE):
            super().compare_cpp(stem)

    def test_actual_node_tick_and_status_bytes(self):
        self._diagnostic_trace_name = 'recovery-node'
        try:
            with patch.object(stage3_trace, 'BASELINE', BASELINE):
                super().test_actual_node_tick_and_status_bytes()
            self._correct_inherited_summary('node.json', 'recovery-node-comparison.json')
        finally:
            self._diagnostic_trace_name = None


class Stage2PoweredNodeEquivalenceTests(
    _DiagnosticComparisonMixin, powered_trace.PoweredNodeEquivalenceTests
):
    """真实节点 RC 起飞/定高/降落/再准备，wire 浮点逐位对照。"""

    def test_powered_rc_lifecycle_and_all_sink_fields_match_baseline(self):
        self._diagnostic_trace_name = 'powered-node'
        evidence = os.environ.get(EVIDENCE_ENV, '/tmp/fc01-stage2-differential')
        try:
            with patch.object(powered_trace, 'BASELINE', BASELINE), patch.dict(
                os.environ, {'FC01_STAGE2_EVIDENCE': evidence}
            ):
                super().test_powered_rc_lifecycle_and_all_sink_fields_match_baseline()
            self._correct_inherited_summary('powered-node.json', 'powered-node-comparison.json')
        finally:
            self._diagnostic_trace_name = None


if __name__ == '__main__':
    unittest.main()
