from pathlib import Path
import unittest
import yaml

ROOT=Path(__file__).resolve().parents[1]

class ReviewPackageTests(unittest.TestCase):
    def test_default_attitude_config_cannot_authorize_arm_or_takeoff(self):
        p=yaml.safe_load((ROOT/'config/attitude_altitude.yaml').read_text())['fc01_system']['ros__parameters']
        for key in ('flight_actions_enabled','arm_enabled','takeoff_enabled','flight_parameters_confirmed',
                    'bench_arm_only_enabled','rc_takeover_confirmed'):
            self.assertIs(p.get(key),False,key)

    def test_delivery_contains_required_local_tools(self):
        for name in ('fcctl.py','deploy.py','socket_safety.py','r6_preflight_audit.py','s10_timeline.py','start.py'):
            self.assertTrue((ROOT/'tools'/name).is_file(),name)

if __name__=='__main__':unittest.main()
