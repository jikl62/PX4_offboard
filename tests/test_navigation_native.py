"""Run real C++ contracts; cache only artifacts keyed by actual source/header bytes."""
from pathlib import Path
import subprocess
import unittest

ROOT = Path(__file__).resolve().parents[1]

class NativeNavigationTests(unittest.TestCase):
    def test_cpp_regressions_and_rc_navigation(self):
        result = subprocess.run(['python3', str(ROOT/'tests/run_native_cpp.py')], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, (result.stdout+result.stderr)[-14000:])

    def test_actual_ros_adapter_syntax_with_pinned_message_api(self):
        result = subprocess.run(['python3', str(ROOT/'native/tests/adapter_compile_check.py')],
                                capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stdout+result.stderr)

if __name__ == '__main__':
    unittest.main()
