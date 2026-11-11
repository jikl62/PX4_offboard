"""用相同外部事件比较 5198d33、独立退出修复与当前职责拆分的严格可观察行为。"""

import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
SOURCE = 'native/tools/vision_odometry_adapter.py'
BASELINES = ('5198d33f25aaaf2abc7e4b078169ae84d5c4b2c4', 'df735d6')


class VisionRefactorEquivalenceTests(unittest.TestCase):
    """比较配置声明、QoS、回调事件消费、消息/日志顺序与拒绝文本，不用数值容差。"""

    def test_main_external_contract_matches_both_baselines(self):
        """六种确定性场景的全部 JSON 轨迹严格相同；信号策略由独立退出回归验证。"""
        if shutil.which('git') is None or not (ROOT / '.git').exists():
            self.skipTest('historical differential needs a Git checkout and baseline objects')
        traces = {}
        with tempfile.TemporaryDirectory(prefix='fc01-stage2-vision-diff-') as directory:
            candidates = [('current', ROOT / SOURCE)]
            for revision in BASELINES:
                probe = subprocess.run(
                    ['git', 'cat-file', '-e', revision + ':' + SOURCE],
                    cwd=ROOT,
                    capture_output=True,
                )
                if probe.returncode:
                    self.skipTest('baseline object unavailable: ' + revision)
                target = Path(directory) / (revision + '.py')
                target.write_bytes(
                    subprocess.check_output(['git', 'show', revision + ':' + SOURCE], cwd=ROOT)
                )
                candidates.append((revision, target))
            for name, source in candidates:
                traces[name] = {}
                for scenario in (
                    'callbacks',
                    'missing_identity',
                    'missing_topic',
                    'bad_depth',
                    'bad_pair_timeout',
                    'sim_time',
                ):
                    with self.subTest(revision=name, scenario=scenario):
                        result = subprocess.check_output(
                            [
                                sys.executable,
                                str(ROOT / 'tests/stage2_vision_trace.py'),
                                str(source),
                                scenario,
                            ],
                            cwd=ROOT,
                            text=True,
                        )
                        traces[name][scenario] = json.loads(result)
            for revision in BASELINES:
                for scenario, expected in traces[revision].items():
                    with self.subTest(revision=revision, scenario=scenario):
                        self.assertEqual(expected, traces['current'][scenario])
        publications = [
            entry for entry in traces['current']['callbacks']['details'] if entry[0] == 'publish'
        ]
        self.assertGreaterEqual(len(publications), 5)
        self.assertTrue(any(entry[2]['valid_fields'] & 71 == 71 for entry in publications))
        self.assertTrue(any(entry[2]['valid_fields'] == 0 for entry in publications))
        self.assertIsNone(traces['current']['callbacks']['error'])


if __name__ == '__main__':
    unittest.main()
