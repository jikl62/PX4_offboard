# 离线预检审计回归：检查参数门、双sink本地提交、时序连续性与未知证据。
# 所有事件均是内存字典；本地提交通过也不得宣称PX4已接收。
import importlib.util
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('r6_audit', ROOT / 'tools/r6_preflight_audit.py')
audit = importlib.util.module_from_spec(spec)
spec.loader.exec_module(audit)


# 用最小反例验证审计器对缺证据、不完整提交和时间异常均不放行。
class PreflightAuditTests(unittest.TestCase):
    # 真实待审配置只允许关闭动作门的只读检查，powered_flight_go仍须为false。
    def test_closed_gates_are_for_read_only_checks(self):
        result = audit.check_config(ROOT / 'config/R6_REAL_FC01_PENDING_REVIEW.yaml')
        self.assertTrue(result['closed_gate_check_pass'])
        self.assertFalse(result['powered_flight_go'])

    # 字符串false不是布尔false，不能通过关闭门检查。
    def test_truthy_string_cannot_pass_as_disabled(self):
        p = {
            'flight_actions_enabled': 'false',
            'arm_enabled': False,
            'takeoff_enabled': False,
            'flight_parameters_confirmed': False,
        }
        self.assertFalse(audit.audit_parameters(p)['closed_gate_check_pass'])

    # 目标已提交但心跳未提交的半次tick必须计数并否决双sink完整性。
    def test_half_published_tick_does_not_count_as_complete(self):
        rows = [
            {
                'kind': 'phase_event',
                'monotonic_s': 1.0,
                'target_local_commit': True,
                'heartbeat_local_commit': False,
            },
            {
                'kind': 'phase_event',
                'monotonic_s': 1.05,
                'target_local_commit': True,
                'heartbeat_local_commit': True,
            },
        ]
        result = audit.check_events(rows)
        self.assertEqual(result['partial_sink_ticks'], 1)
        self.assertFalse(result['local_dual_sink_check_pass'])

    # 注入超过0.3秒的间隔与倒退时间，既报告异常也保持PX4接收未证实。
    def test_gap_and_time_regression_are_rejected(self):
        rows = [
            {
                'kind': 'phase_event',
                'monotonic_s': t,
                'target_local_commit': True,
                'heartbeat_local_commit': True,
            }
            for t in [1.0, 1.5, 1.4]
        ]
        result = audit.check_events(rows)
        self.assertGreater(result['max_local_heartbeat_gap_s'], 0.3)
        self.assertFalse(result['monotonic_order_valid'])
        self.assertFalse(result['px4_receipt_proven'])

    # 空事件流缺少提交证据，不能作为成功。
    def test_empty_events_are_unknown(self):
        self.assertFalse(audit.check_events([])['local_dual_sink_check_pass'])

    # 即使ownership文字描述PX4待接管，已解锁Offboard末尾静默仍须拒绝通过。
    def test_healthy_offboard_silent_tail_cannot_pass(self):
        rows = [
            {
                'kind': 'phase_event',
                'monotonic_s': t,
                'target_local_commit': i == 0,
                'heartbeat_local_commit': i == 0,
                'armed': True,
                'nav_state': 14,
                'ownership': 'PX4_FAILSAFE_PENDING_UNKNOWN_AIRBORNE',
            }
            for i, t in enumerate([1.0, 1.1, 1.5])
        ]
        self.assertFalse(audit.check_events(rows)['local_dual_sink_check_pass'])

    # 同一行提交标志为true也不能掩盖sink_exception。
    def test_sink_exception_is_not_a_local_pass(self):
        row = {
            'kind': 'phase_event',
            'monotonic_s': 1.0,
            'target_local_commit': True,
            'heartbeat_local_commit': True,
            'sink_exception': True,
        }
        self.assertFalse(audit.check_events([row])['local_dual_sink_check_pass'])


if __name__ == '__main__':
    unittest.main()
