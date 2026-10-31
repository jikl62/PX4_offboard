import importlib.util
from pathlib import Path
import unittest
ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('r6_audit', ROOT/'tools/r6_preflight_audit.py')
audit=importlib.util.module_from_spec(spec)
spec.loader.exec_module(audit)
class PreflightAuditTests(unittest.TestCase):
    def test_closed_gates_are_for_read_only_checks(self):
        result=audit.check_config(ROOT/'config/R6_REAL_FC01_PENDING_REVIEW.yaml')
        self.assertTrue(result['closed_gate_check_pass'])
        self.assertFalse(result['powered_flight_go'])
    def test_truthy_string_cannot_pass_as_disabled(self):
        p={'flight_actions_enabled':'false','arm_enabled':False,'takeoff_enabled':False,
           'flight_parameters_confirmed':False}
        self.assertFalse(audit.audit_parameters(p)['closed_gate_check_pass'])
    def test_half_published_tick_does_not_count_as_complete(self):
        rows=[{'kind':'phase_event','monotonic_s':1.0,'target_local_commit':True,'heartbeat_local_commit':False},
              {'kind':'phase_event','monotonic_s':1.05,'target_local_commit':True,'heartbeat_local_commit':True}]
        result=audit.check_events(rows)
        self.assertEqual(result['partial_sink_ticks'],1)
        self.assertFalse(result['local_dual_sink_check_pass'])
    def test_gap_and_time_regression_are_rejected(self):
        rows=[{'kind':'phase_event','monotonic_s':t,'target_local_commit':True,'heartbeat_local_commit':True}
              for t in [1.0,1.5,1.4]]
        result=audit.check_events(rows)
        self.assertGreater(result['max_local_heartbeat_gap_s'],.3)
        self.assertFalse(result['monotonic_order_valid'])
        self.assertFalse(result['px4_receipt_proven'])
    def test_empty_events_are_unknown(self):
        self.assertFalse(audit.check_events([])['local_dual_sink_check_pass'])
    def test_healthy_offboard_silent_tail_cannot_pass(self):
        rows=[{'kind':'phase_event','monotonic_s':t,'target_local_commit':i==0,'heartbeat_local_commit':i==0,
               'armed':True,'nav_state':14,'ownership':'PX4_FAILSAFE_PENDING_UNKNOWN_AIRBORNE'}
              for i,t in enumerate([1.0,1.1,1.5])]
        self.assertFalse(audit.check_events(rows)['local_dual_sink_check_pass'])
    def test_sink_exception_is_not_a_local_pass(self):
        row={'kind':'phase_event','monotonic_s':1.0,'target_local_commit':True,
             'heartbeat_local_commit':True,'sink_exception':True}
        self.assertFalse(audit.check_events([row])['local_dual_sink_check_pass'])
if __name__=='__main__':unittest.main()
