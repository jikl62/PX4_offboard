import json
from pathlib import Path
import subprocess
import unittest

ROOT=Path(__file__).resolve().parents[1]
class AuthorityHarnessTests(unittest.TestCase):
    def test_deterministic_real_core_and_sink_evidence(self):
        command=['bash',str(ROOT/'scripts/test_authority_harness.sh')]
        first=subprocess.check_output(command)
        second=subprocess.check_output(command)
        self.assertEqual(first,second)
        data=json.loads(first);self.assertFalse(data['hardware_authorized'])
        self.assertEqual(data['summary'],{'total':24,'passed':24,'failed':0})
        cases={x['name']:x for x in data['scenarios']}
        for name in ['rc_synthetic_arm_and_thrust','autonomous_without_rc_synthetic_arm_and_thrust']:
            self.assertEqual(cases[name]['arm_sink_count'],1)
            self.assertGreater(cases[name]['nonzero_target_count'],0)
        for name in ['unset_takeoff_denied','rc_missing_link_denied','rc_contract_unconfirmed','autonomous_contract_unconfirmed','pending_arm_disconnect']:
            self.assertEqual(cases[name]['arm_sink_count'],0)
            self.assertEqual(cases[name]['nonzero_target_count'],0)
        for name,item in cases.items():
            self.assertTrue(item['passed'],item)
            self.assertEqual(item['target_sink_count'],item['mode_sink_count'])
            self.assertLessEqual(item['offboard_command_count'],1)
if __name__=='__main__':unittest.main()
