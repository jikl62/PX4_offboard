# 本地进程信号回归：编译故障夹具，分别验证优雅停止、SIGKILL与崩溃的可证明边界。
# 只检查本地恢复目标和心跳，外部PX4恢复保持UNVERIFIED。
"""Local OS signals only; external PX4 recovery deliberately UNVERIFIED."""
import json
import resource
import signal
import subprocess
import tempfile
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]


# 每组只编译一份临时ELF，单个场景使用独立子进程。
class ProcessFailureTests(unittest.TestCase):
    # 用真实头文件构建r6_fault_child，全部产物置于临时目录。
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory(prefix='r6_signal_fixture_')
        cls.exe = Path(cls.tmp.name) / 'fault_child'
        subprocess.run(
            [
                'g++',
                '-std=c++17',
                '-Wall',
                '-Wextra',
                '-Werror',
                '-pedantic',
                '-I',
                str(ROOT / 'src/jikl_fc01_system/include'),
                '-I',
                str(ROOT / 'tests'),
                str(ROOT / 'tests/r6_fault_child.cpp'),
                '-o',
                str(cls.exe),
            ],
            check=True,
        )

    # 测试组结束清理临时二进制，避免留下与下一次源码不匹配的产物。
    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    # 关闭core转储并等到READY后返回，确保信号打在已初始化的夹具上。
    def child(self, scenario=None):
        def no_core():
            resource.setrlimit(resource.RLIMIT_CORE, (0, 0))

        p = subprocess.Popen(
            [str(self.exe)] + ([scenario] if scenario else []),
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            preexec_fn=no_core,
        )
        self.assertEqual(p.stdout.readline().strip(), 'READY')
        return p

    # 优雅停止超过恢复预算后仍须保持目标和心跳，不得虚构PX4后备接管成功。
    def graceful(self, sig):
        p = self.child()
        p.send_signal(sig)
        out, err = p.communicate(timeout=8)
        self.assertEqual(p.returncode, 0, err)
        rows = [json.loads(line) for line in out.splitlines()]
        active = [r for r in rows if r['shutting_down'] and r['heartbeat']]
        self.assertGreater(len(active), 3)
        self.assertTrue(all(r['target'] for r in active))
        self.assertEqual(rows[-1]['phase'], 'RECOVERY_DESCENT')
        self.assertTrue(rows[-1]['recovery_budget_exhausted'])
        self.assertTrue(all(r['target'] and r['heartbeat'] for r in rows))
        self.assertFalse(rows[-1]['px4_fallback_verified'])

    # SIGINT应保留健康恢复控制，复用共同证据断言。
    def test_sigint_retains_healthy_custody_after_budget(self):
        self.graceful(signal.SIGINT)

    # SIGTERM应与交互停止保持相同恢复责任。
    def test_sigterm_retains_healthy_custody_after_budget(self):
        self.graceful(signal.SIGTERM)

    # SIGKILL不可能完成本机恢复，输出中不得宣称px4_fallback_verified。
    def test_sigkill_cannot_claim_companion_recovery(self):
        p = self.child()
        p.kill()
        out, err = p.communicate(timeout=8)
        self.assertEqual(p.returncode, -signal.SIGKILL, err)
        self.assertFalse(any(json.loads(x)['px4_fallback_verified'] for x in out.splitlines()))

    # 主动崩溃应是SIGABRT且不产生虚假的恢复日志。
    def test_crash_cannot_claim_companion_recovery(self):
        p = self.child('crash')
        out, err = p.communicate(timeout=8)
        self.assertEqual(p.returncode, -signal.SIGABRT, err)
        self.assertEqual(out, '')


if __name__ == '__main__':
    unittest.main()
