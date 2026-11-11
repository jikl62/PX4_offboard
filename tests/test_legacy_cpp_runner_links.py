"""用真实编译器验证两个旧Shell入口的逐测试链接合同；不安装或启动控制器。"""

import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = ('test.sh', 'r62_cpp_regressions.sh')


class LegacyCppRunnerLinkTests(unittest.TestCase):
    """小目录保留真实Shell循环与日志资源测试；历史回放/ROS编译另由原集合验收。"""

    def run_fixture(self, script, *, failing=False):
        # 每次独立目录：复制入口、唯一生产头与真实故障注入测试，不触及用户构建/安装目录。
        with tempfile.TemporaryDirectory(prefix='fc01-legacy-runner-') as temporary:
            folder = Path(temporary)
            native = folder / 'native'
            scripts = native / 'scripts'
            tests = native / 'tests'
            include = native / 'src/jikl_fc01_system/include/jikl_fc01_system'
            historical = tests / 'c_independent_repro'
            for path in (scripts, tests, include, historical, folder / 'bin'):
                path.mkdir(parents=True, exist_ok=True)
            shutil.copy2(ROOT / 'native/scripts' / script, scripts / script)
            shutil.copy2(
                ROOT / 'native/src/jikl_fc01_system/include/jikl_fc01_system/async_log.hpp',
                include / 'async_log.hpp',
            )
            shutil.copy2(
                ROOT / 'native/tests/test_stage3_async_log_resources.cpp',
                tests / 'test_stage3_async_log_resources.cpp',
            )
            (tests / 'test_ordinary.cpp').write_text(
                'int main() { return ' + ('17' if failing else '0') + '; }\n'
            )
            # 此夹具只验正向编译循环。后续入口留有可观察标记，不把伪造历史输入计作回放通过。
            for name in (
                'C_M1_COUNTEREXAMPLES.cpp',
                'C_R2_NEW_INTERLEAVINGS.cpp',
                'C_R2_SLEW_AND_ACK.cpp',
            ):
                (historical / name).write_text('int main() { return 0; }\n')
            for path in (
                tests / 'replay_c11_c12.py',
                tests / 'adapter_compile_check.py',
                scripts / 'r62_failure_replay.py',
            ):
                path.write_text("print('FIXTURE_POST_CPP_REACHED')\n")

            # 透明记录实际g++参数后exec真实编译器，不伪造编译结果或跳过故障注入断言。
            record = folder / 'compiler.jsonl'
            wrapper = folder / 'bin/g++'
            wrapper.write_text(
                '#!' + shutil.which('python3') + '\n'
                'import json,os,sys\n'
                'with open(os.environ["FC01_TEST_COMPILER_LOG"],"a") as out:\n'
                ' out.write(json.dumps(sys.argv[1:])+"\\n")\n'
                'os.execv(os.environ["FC01_TEST_REAL_COMPILER"],'
                '[os.environ["FC01_TEST_REAL_COMPILER"],*sys.argv[1:]])\n'
            )
            wrapper.chmod(0o700)
            environment = {
                **os.environ,
                'PATH': str(folder / 'bin') + os.pathsep + os.environ['PATH'],
                'FC01_TEST_COMPILER_LOG': str(record),
                'FC01_TEST_REAL_COMPILER': shutil.which('g++'),
                'PYTHONDONTWRITEBYTECODE': '1',
            }
            result = subprocess.run(
                ['bash', str(scripts / script)],
                env=environment,
                capture_output=True,
                text=True,
                timeout=60,
            )
            commands = [json.loads(line) for line in record.read_text().splitlines()]
            return result, commands

    def test_actual_resource_injection_links_only_to_its_test(self):
        """两个原入口都能执行真实资源注入测试，普通程序及历史夹具不获wrap参数。"""
        for script in SCRIPTS:
            with self.subTest(script=script):
                result, commands = self.run_fixture(script)
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
                self.assertIn('FIXTURE_POST_CPP_REACHED', result.stdout)
                special = [
                    argv
                    for argv in commands
                    if any(arg.endswith('/test_stage3_async_log_resources.cpp') for arg in argv)
                ]
                self.assertEqual(len(special), 1)
                for argv in commands:
                    links = {arg for arg in argv if arg.startswith('-Wl,--wrap=') or arg == '-ldl'}
                    self.assertEqual(
                        links,
                        (
                            {'-Wl,--wrap=fcntl', '-Wl,--wrap=write', '-ldl'}
                            if argv == special[0]
                            else set()
                        ),
                    )

    def test_failing_ordinary_test_still_stops_the_entry(self):
        """修复链接不能吞掉普通测试非零退出；未通过就不可执行后续步骤。"""
        for script in SCRIPTS:
            with self.subTest(script=script):
                result, commands = self.run_fixture(script, failing=True)
                self.assertEqual(result.returncode, 17, result.stdout + result.stderr)
                self.assertNotIn('FIXTURE_POST_CPP_REACHED', result.stdout)
                self.assertEqual(len(commands), 1)


if __name__ == '__main__':
    unittest.main()
