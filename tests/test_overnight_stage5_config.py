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


class VisionCandidateNumericTests(unittest.TestCase):
    """JSON 整数可比 double 大；拒绝它时仍须遵守配置错误合同。"""

    def base(self):
        return navigation_service.navigation_profile(
            yaml.safe_load((ROOT / 'tests/fixtures/base.yaml').read_text()), mode_slot=1
        )

    def test_oversized_json_integers_raise_configuration_error_with_cause(self):
        # 遍历实际参数白名单，不只验证恰好被复现的 max_speed；坏输入本身不能被修改。
        for name in vision_settings.DEFAULTS:
            for value in (10**400, -(10**400)):
                with self.subTest(parameter=name, positive=value > 0):
                    selected = {'enabled': False, 'parameters': {name: value}}
                    before = copy.deepcopy(selected)
                    with self.assertRaises(navigation_service.ControlError) as error:
                        vision_settings.apply_settings(self.base(), selected)
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


if __name__ == '__main__':
    unittest.main()
