# 交付目录静态回归：检查默认配置动作门关闭，必需本地工具齐全。
# 文件存在与配置检查只是交付完整性证据，不能代替上板或实际飞行验收。
from pathlib import Path
import unittest
import yaml

ROOT=Path(__file__).resolve().parents[1]

# 只读取包内配置和工具路径，不执行部署或控制动作。
class ReviewPackageTests(unittest.TestCase):
    # 默认姿态定高配置的动作门、参数确认与台架/RC确认必须全部为严格false。
    def test_default_attitude_config_cannot_authorize_arm_or_takeoff(self):
        p=yaml.safe_load((ROOT/'config/attitude_altitude.yaml').read_text())['fc01_system']['ros__parameters']
        for key in ('flight_actions_enabled','arm_enabled','takeoff_enabled','flight_parameters_confirmed',
                    'bench_arm_only_enabled','rc_takeover_confirmed'):
            self.assertIs(p.get(key),False,key)

    # 检查部署、socket策略、预检、时间线与启动工具都包含在源码交付内。
    def test_delivery_contains_required_local_tools(self):
        for name in ('fcctl.py','deploy.py','socket_safety.py','r6_preflight_audit.py','s10_timeline.py','start.py'):
            self.assertTrue((ROOT/'tools'/name).is_file(),name)

if __name__=='__main__':unittest.main()
