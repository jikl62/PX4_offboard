"""复用已审查的真实控制驱动，以 f06715a 为本轮不可变基线重新做精确差分。

只改变被编译的源码基线，不复制控制算法、不修改发布时序，也不对权限或浮点放宽容差。
新组合测试同时在旧实现运行，防止把新增测试自身的假设当成新修复。
"""

import os
from pathlib import Path
from unittest.mock import patch

import test_stage2_core_equivalence as core_trace
import test_stage2_powered_node_equivalence as powered_trace


BASELINE = 'f06715a70c6c070ecce25551ecb5de535e51b7a3'


class Stage3CoreEquivalenceTests(core_trace.CoreBehaviorEquivalenceTests):
    """同一组 Core/事务/节点输入和新的故障组合，分别编译两份生产实现。"""

    @classmethod
    def setUpClass(cls):
        """仅在准备期间替换历史驱动的基线常量；不改变其他 Stage2 测试的基线。"""
        with patch.object(core_trace, 'BASELINE', BASELINE):
            super().setUpClass()
        cls.evidence = Path(os.environ.get('FC01_STAGE3_TRACE_DIR', cls.work / 'stage3'))
        cls.evidence.mkdir(parents=True, exist_ok=True)

    def compare_cpp(self, stem):
        """调用既有严格字节比较器，并令证据中的基线身份与实际编译输入相同。"""
        with patch.object(core_trace, 'BASELINE', BASELINE):
            super().compare_cpp(stem)

    def test_actual_node_tick_and_status_bytes(self):
        """节点源码来自本轮基线；证据中的基线身份也必须在整个调用期间保持正确。"""
        with patch.object(core_trace, 'BASELINE', BASELINE):
            super().test_actual_node_tick_and_status_bytes()

    def test_publication_observation_combinations(self):
        """三个观察点的 512 种证据组合与 10 种异常注入必须保持相同结果。"""
        self.compare_cpp('test_stage3_publication_combinations')

    def test_unauthorized_startup_and_latched_hazards(self):
        """256 种无本轮授权启动、25 种危险事件次序、1024 种 FIFO 突发序列。"""
        self.compare_cpp('test_stage3_state_invariants')


class Stage3PoweredNodeEquivalenceTests(powered_trace.PoweredNodeEquivalenceTests):
    """真实节点的人工解锁、1.5 m 平台、降落及恢复，所有 sink 仍为内存替身。"""

    def test_powered_rc_lifecycle_and_all_sink_fields_match_baseline(self):
        """仅将既有完整节点驱动的不可变输入基线切为本轮 f06715a。"""
        evidence = os.environ.get('FC01_STAGE3_TRACE_DIR')
        updates = {'FC01_STAGE2_EVIDENCE': evidence} if evidence else {}
        with patch.object(powered_trace, 'BASELINE', BASELINE), patch.dict(os.environ, updates):
            super().test_powered_rc_lifecycle_and_all_sink_fields_match_baseline()
