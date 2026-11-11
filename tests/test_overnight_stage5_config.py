"""视觉候选配置的错误边界；只用本地临时文件，不导入 ROS 或启动服务。"""

import copy
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import yaml

import navigation_service
import vision_settings

ROOT = Path(__file__).resolve().parents[1]


def base_configuration():
    """每次独立生成关闭程序解锁的原候选，避免原地参数更新污染其他场景。"""
    return navigation_service.navigation_profile(
        yaml.safe_load((ROOT / 'tests/fixtures/base.yaml').read_text()), mode_slot=1
    )


class VisionCandidateNumericTests(unittest.TestCase):
    """JSON 整数可比 double 大；拒绝它时仍须遵守配置错误合同。"""

    def test_oversized_json_integers_raise_configuration_error_with_cause(self):
        # 遍历实际参数白名单，不只验证恰好被复现的 max_speed；坏输入本身不能被修改。
        for name in vision_settings.DEFAULTS:
            for value in (10**400, -(10**400)):
                with self.subTest(parameter=name, positive=value > 0):
                    selected = {'enabled': False, 'parameters': {name: value}}
                    before = copy.deepcopy(selected)
                    with self.assertRaises(navigation_service.ControlError) as error:
                        vision_settings.apply_settings(base_configuration(), selected)
                    self.assertIn(name, str(error.exception))
                    self.assertIsInstance(error.exception.__cause__, OverflowError)
                    self.assertEqual(selected, before)

    def test_exporter_returns_two_and_retains_existing_output_on_numeric_rejection(self):
        # CLI 拒绝发生在写候选文件前；已有文件/输入文件不能因坏配置被覆盖。
        with tempfile.TemporaryDirectory(prefix='fc01-vision-numeric-') as directory:
            folder = Path(directory)
            selected = folder / 'vision.json'
            selected.write_text(json.dumps({'parameters': {'max_speed': 10**400}}))
            original_settings = selected.read_bytes()
            output = folder / 'candidate.yaml'
            output.write_text('previous candidate retained\n')
            original_output = output.read_bytes()
            result = subprocess.run(
                [
                    sys.executable,
                    str(ROOT / 'vision_config.py'),
                    '--settings',
                    str(selected),
                    '--output',
                    str(output),
                ],
                capture_output=True,
                text=True,
                timeout=10,
            )
            self.assertEqual(result.returncode, 2, result.stderr)
            self.assertIn('配置生成失败', result.stderr)
            self.assertIn('max_speed', result.stderr)
            self.assertNotIn('Traceback', result.stderr)
            self.assertEqual(output.read_bytes(), original_output)
            self.assertEqual(selected.read_bytes(), original_settings)


class VisionCandidateTopicTests(unittest.TestCase):
    """视觉启用候选应先满足生产节点已经存在的双类型话题约束。"""

    def test_enabled_candidate_rejects_state_and_command_on_one_topic(self):
        selected = {
            'enabled': True,
            'state_topic': '/demo/vision_input',
            'command_topic': '/demo/vision_input',
        }
        before = copy.deepcopy(selected)
        with self.assertRaises(navigation_service.ControlError) as error:
            vision_settings.apply_settings(base_configuration(), selected)
        self.assertIn('不能相同', str(error.exception))
        self.assertEqual(selected, before)

    def test_disabled_candidate_keeps_existing_topic_compatibility(self):
        # 原生未启用视觉时不会构造视觉订阅；本次不扩大关闭路径的拒绝范围。
        config = base_configuration()
        result = vision_settings.apply_settings(
            config,
            {
                'enabled': False,
                'state_topic': '/demo/unused',
                'command_topic': '/demo/unused',
            },
        )
        self.assertIs(result, config)
        parameters = config['fc01_system']['ros__parameters']
        self.assertIs(parameters['visual_navigation_enabled'], False)
        self.assertEqual(parameters['visual_state_topic'], parameters['visual_command_topic'])

    def test_exporter_rejects_topic_collision_before_writing_candidate(self):
        with tempfile.TemporaryDirectory(prefix='fc01-vision-topics-') as directory:
            folder = Path(directory)
            selected = folder / 'vision.json'
            selected.write_text(
                json.dumps(
                    {
                        'enabled': True,
                        'state_topic': '/demo/same',
                        'command_topic': '/demo/same',
                    }
                )
            )
            output = folder / 'candidate.yaml'
            output.write_text('retained previous candidate\n')
            before = output.read_bytes()
            result = subprocess.run(
                [
                    sys.executable,
                    str(ROOT / 'vision_config.py'),
                    '--settings',
                    str(selected),
                    '--output',
                    str(output),
                ],
                capture_output=True,
                text=True,
                timeout=10,
            )
            self.assertEqual(result.returncode, 2, result.stderr)
            self.assertIn('不能相同', result.stderr)
            self.assertNotIn('Traceback', result.stderr)
            self.assertEqual(output.read_bytes(), before)


if __name__ == '__main__':
    unittest.main()
