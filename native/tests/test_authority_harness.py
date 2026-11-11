# 真实权限核心的离线场景汇总：比较两次输出，检查24个场景的命令与双sink证据。
# 合成RC/自主授权只在测试夹具内生效；hardware_authorized必须仍为false。
import json
from pathlib import Path
import subprocess
import unittest

ROOT=Path(__file__).resolve().parents[1]
# 按场景名称检查成功与拒绝路径，避免仅依据总通过数判断。
class AuthorityHarnessTests(unittest.TestCase):
    # 同一输入两次执行必须逐字节一致；拒绝场景不得产生Arm或非零推力，目标与心跳提交次数须一致。
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
