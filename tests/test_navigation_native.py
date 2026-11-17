# 导航相关原生回归聚合入口：运行真实 C++ 单测和固定 px4_msgs API 的编译探针。
# 这两项仅在主机离线执行，不创建生产 ROS 节点或连接飞控。
"""Run real C++ contracts; cache only artifacts keyed by actual source/header bytes."""
from pathlib import Path
import subprocess
import unittest

ROOT = Path(__file__).resolve().parents[1]


class NativeNavigationTests(unittest.TestCase):
    # 调用内容寻址 C++ runner，必须实际执行所有断言程序。
    def test_cpp_regressions_and_rc_navigation(self):
        result = subprocess.run(
            ['python3', str(ROOT / 'tests/run_native_cpp.py')], capture_output=True, text=True
        )
        self.assertEqual(result.returncode, 0, (result.stdout + result.stderr)[-14000:])

    # 用实际固定消息 API 编译适配边界，避免只靠 Python 假消息确认兼容。
    def test_actual_ros_adapter_syntax_with_pinned_message_api(self):
        result = subprocess.run(
            ['python3', str(ROOT / 'native/tests/adapter_compile_check.py')],
            capture_output=True,
            text=True,
        )
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)


if __name__ == '__main__':
    unittest.main()
