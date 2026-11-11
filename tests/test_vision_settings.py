# 视觉配置是显式候选合同：默认关闭，开启参数也不得扩大原 RC/姿态安全包络。
# 分别检查 Python 设置解析、真实 C++ 参数验证和离线导出，均不授予视觉飞行权限。
import copy
import importlib
import json
from pathlib import Path
import tempfile
import subprocess
import sys
import unittest

import yaml
import navigation_service
from test_native_profile import NativeProfileContract

ROOT=Path(__file__).resolve().parents[1]


# 确保严格字段/类型验证，视觉关闭时原起飞参数保持。
class VisionSettingsTests(unittest.TestCase):
    # 加载实际 vision_settings 模块，验证当前源码接口。
    def api(self):
        self.assertTrue((ROOT/'vision_settings.py').exists(), 'vision configuration adapter missing')
        return importlib.import_module('vision_settings')

    # 每次读取独立基线配置，避免前一项参数修改污染后一项。
    def base(self):
        return navigation_service.navigation_profile(yaml.safe_load((ROOT/'tests/fixtures/base.yaml').read_text()),mode_slot=1)

    # 没有显式视觉设置时只写关闭标记，原高度/推力/RC 数值保持。
    def test_default_off_preserves_existing_flight_parameters(self):
        m=self.api();config=self.base();before=copy.deepcopy(config)
        m.apply_settings(config,{'enabled':False})
        p=config['fc01_system']['ros__parameters']
        self.assertIs(p['visual_navigation_enabled'],False)
        for name,value in before['fc01_system']['ros__parameters'].items():
            self.assertEqual(p[name],value,name)

    # 关闭视觉不应要求原控制器放宽更保守的高度/倾角限制。
    def test_disabled_visual_accepts_stricter_existing_flight_envelope(self):
        m=self.api();config=self.base();p=config['fc01_system']['ros__parameters']
        p.update(aa_max_height=2.,aa_max_tilt_deg=3.,aa_max_climb=.1,aa_max_descent=.1)
        m.apply_settings(config,{'enabled':False})
        self.assertFalse(p['visual_navigation_enabled'])
        self.assertEqual(p['aa_max_height'],2.)

    # 明确话题和候选限幅必须准确写入原生参数，不使用隐式视觉默认控制值。
    def test_explicit_topics_and_candidate_limits_reach_generated_parameters(self):
        m=self.api();config=self.base()
        m.apply_settings(config,{'enabled':True,'state_topic':'/lab/state','command_topic':'/lab/target',
            'qos_depth':3,'parameters':{'max_speed':.25,'max_tilt_deg':4.}})
        p=config['fc01_system']['ros__parameters']
        self.assertTrue(p['visual_navigation_enabled'])
        self.assertEqual(p['visual_state_topic'],'/lab/state')
        self.assertEqual(p['visual_command_topic'],'/lab/target')
        self.assertEqual(p['visual_qos_depth'],3)
        self.assertEqual(p['visual_max_speed'],.25)
        self.assertEqual(p['visual_max_tilt_deg'],4.)
        self.assertFalse(p['arm_enabled'])

    # 拼错字段、类型或非法数值应明确拒绝，不能悄悄按默认继续。
    def test_invalid_or_misspelled_config_is_not_silently_ignored(self):
        m=self.api()
        for settings in ({'enabled':'true'},{'enable':True},{'state_topic':'relative'},
                         {'command_topic':'/fmu/in/vehicle_command'},{'state_topic':'/a/../b'},
                         {'qos_depth':True},{'enabled':True,'parameters':{'max_tilt_deg':11.}},
                         {'parameters':{'max_speed':float('nan')}},{'parameters':{'gian':1}}):
            with self.subTest(settings=settings),self.assertRaises(navigation_service.ControlError):
                m.apply_settings(self.base(),settings)

    # 缺设置文件代表视觉关闭；已有合法文件须实际消费而非始终忽略。
    def test_missing_file_is_disabled_and_existing_file_is_consumed(self):
        m=self.api()
        with tempfile.TemporaryDirectory() as d:
            entry=Path(d)
            config=self.base();m.load(config,entry)
            self.assertFalse(config['fc01_system']['ros__parameters']['visual_navigation_enabled'])
            (entry/'vision_settings.json').write_text(json.dumps({'enabled':True,'parameters':{'max_speed':.2}}))
            config=self.base();m.load(config,entry)
            self.assertEqual(config['fc01_system']['ros__parameters']['visual_max_speed'],.2)


# 由真实 Config::validate 复核视觉候选与现有 RC 路线/倾角包络的约束。
class VisionNativeConfigTests(NativeProfileContract):
    # 从现有基线派生显式候选视觉参数，供原生拒绝边界测试。
    def parameters_visual(self):
        return navigation_service.navigation_profile(yaml.safe_load((ROOT/'tests/fixtures/base.yaml').read_text()),mode_slot=1)['fc01_system']['ros__parameters']

    # 视觉最大倾角大于原高度控制上限时必须拒绝。
    def test_visual_configuration_cannot_expand_original_tilt_envelope(self):
        p=self.parameters_visual();p.update(visual_navigation_enabled=True,visual_max_tilt_deg=11.)
        self.assertNotEqual(self.run_native(p).returncode,0)

    # 视觉不能绕过已存在人工 RC 权限路线单独启用。
    def test_visual_configuration_requires_existing_manual_rc_route(self):
        p=self.parameters_visual();p.update(visual_navigation_enabled=True,rc_navigation_enabled=False)
        self.assertNotEqual(self.run_native(p).returncode,0)

    # 禁用视觉时更保守原生配置仍可通过，不被视觉候选范围强制覆盖。
    def test_disabled_visual_preserves_more_conservative_native_configuration(self):
        settings=VisionSettingsTests().api()
        config=navigation_service.navigation_profile(yaml.safe_load((ROOT/'tests/fixtures/base.yaml').read_text()),mode_slot=1)
        p=config['fc01_system']['ros__parameters'];p.update(aa_max_height=2.,aa_max_tilt_deg=3.,aa_max_climb=.1,aa_max_descent=.1)
        settings.apply_settings(config,{'enabled':False})
        result=self.run_native(p)
        self.assertEqual(result.returncode,0,result.stderr)


# 检查离线导出文件与源设置之间关系，不执行部署。
class VisionConfigExportTests(unittest.TestCase):
    # 导出可生成默认关闭或明确候选配置，但输入设置自身不得被写回修改。
    def test_exporter_writes_default_off_or_explicit_candidate_without_changing_settings(self):
        self.assertTrue((ROOT/'vision_config.py').exists(),'offline configuration exporter missing')
        settings_before=(ROOT/'vision_settings.json').read_bytes()
        with tempfile.TemporaryDirectory() as d:
            for enabled in (False,True):
                output=Path(d)/'candidate.yaml'
                args=[sys.executable,str(ROOT/'vision_config.py'),'--output',str(output)]
                if enabled:args.append('--enable-vision')
                run=subprocess.run(args,capture_output=True,text=True)
                self.assertEqual(run.returncode,0,run.stderr)
                p=yaml.safe_load(output.read_text())['fc01_system']['ros__parameters']
                self.assertIs(p['visual_navigation_enabled'],enabled)
                self.assertFalse(p['arm_enabled'])
                self.assertEqual(p['rc_navigation_mode_slot'],1)
                self.assertEqual(p['aa_hover_thrust'],.50)
                self.assertEqual(p['aa_thrust_max'],.80)
        self.assertEqual((ROOT/'vision_settings.json').read_bytes(),settings_before)


if __name__=='__main__':unittest.main()
