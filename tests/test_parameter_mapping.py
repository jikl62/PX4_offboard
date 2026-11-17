"""离线生成器应忽略 C++ 排版，同时完整保留参数到成员的映射。"""

import importlib.util
from pathlib import Path
import re
import shutil
import subprocess
import unittest
from unittest import mock

import vision_replay

ROOT = Path(__file__).resolve().parents[1]
HEADER = 'native/src/jikl_fc01_system/include/jikl_fc01_system/attitude_ros_parameters.hpp'


def native_profile_module():
    # 直接加载同目录测试文件，不要求 tests 是可安装 Python 包。
    spec = importlib.util.spec_from_file_location(
        'fc01_parameter_mapping_native_probe', ROOT / 'tests/test_native_profile.py'
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class ParameterMappingContract(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.current_header = (ROOT / HEADER).read_text()
        cls.native_profile = native_profile_module()

    def generated_sources(self, header):
        # 两条真实生成路径均消费同一份 header；这里只拦截 probe 的编译，
        # 收集最终 C++ 源码进行完整比较，不另复制一份参数解析规则。
        with mock.patch.object(Path, 'read_text', return_value=header):
            replay = vision_replay.parameter_mapping()

        class Probe(self.native_profile.NativeProfileContract):
            pass

        try:
            with (
                mock.patch.object(Path, 'read_text', return_value=header),
                mock.patch.object(self.native_profile.shutil, 'which', return_value='/test/g++'),
                mock.patch.object(
                    self.native_profile.subprocess,
                    'run',
                    return_value=subprocess.CompletedProcess([], 0, '', ''),
                ),
            ):
                Probe.setUpClass()
            native = (Path(Probe.tmp.name) / 'probe.cpp').read_text()
        finally:
            Probe.doClassCleanups()
        return replay, native

    def test_full_original_and_formatted_header_mappings_are_identical(self):
        # 固定到本次中文化前的不可变 Git 对象；比较的是完整生成结果，不是数量近似。
        # 源码安装包不携带 Git 历史，只跳过本项；其它三项仍直接检查当前头文件。
        git = shutil.which('git')
        if git is None:
            self.skipTest('Git unavailable; historical mapping comparison needs repository history')
        repository = subprocess.run(
            [git, '-C', str(ROOT), 'rev-parse', '--show-toplevel'],
            capture_output=True,
            text=True,
            timeout=10,
        )
        if repository.returncode != 0 and not (ROOT / '.git').exists():
            self.skipTest('source installation has no Git repository for the baseline comparison')
        self.assertEqual(repository.returncode, 0, repository.stderr)
        if Path(repository.stdout.strip()).resolve() != ROOT.resolve():
            self.skipTest('source installation does not own the enclosing Git repository')
        baseline = subprocess.run(
            [git, '-C', str(ROOT), 'rev-parse', '--verify', '--quiet', '053eb2c^{commit}'],
            capture_output=True,
            text=True,
            timeout=10,
        )
        if baseline.returncode == 1:
            self.skipTest('Git baseline 053eb2c is absent from this source installation')
        self.assertEqual(baseline.returncode, 0, baseline.stderr)
        # 对象已经确认存在，后续 show 失败属于真实错误，不能以“缺基线”掩盖。
        original = subprocess.run(
            [git, '-C', str(ROOT), 'show', baseline.stdout.strip() + ':' + HEADER],
            capture_output=True,
            text=True,
            timeout=10,
        )
        self.assertEqual(original.returncode, 0, original.stderr)
        expected = self.generated_sources(original.stdout)
        self.assertEqual(len(expected[0].splitlines()), 80)
        self.assertEqual(self.generated_sources(self.current_header), expected)

    def test_token_whitespace_and_line_breaks_preserve_every_mapping(self):
        # 只在现有 token 之间增加空白，包含模板、调用括号、初始化表逗号与取地址符。
        spaced = self.current_header.replace(
            'node.declare_parameter<', 'node.declare_parameter \n < \t'
        )
        spaced = re.sub(r'>\s*\(', ' \t> \n ( \n ', spaced)
        spaced = re.sub(r'\{\s*"', '{ \n "', spaced)
        spaced = re.sub(r'"\s*,\s*&c\.', '" \n , \t& \n c.', spaced)
        spaced = re.sub(r'(c\.[\w.]+)\s*\}', r'\1 \n }', spaced)
        self.assertNotEqual(spaced, self.current_header)
        self.assertEqual(
            self.generated_sources(spaced), self.generated_sources(self.current_header)
        )

    def test_same_count_real_member_changes_are_visible_in_both_outputs(self):
        before = self.generated_sources(self.current_header)
        for old, new in (
            ('c.arm_enabled', 'c.takeoff_enabled'),
            ('&c.vertical.hover', '&c.vertical.maximum'),
        ):
            with self.subTest(member=old):
                self.assertEqual(self.current_header.count(old), 1)
                changed = self.generated_sources(self.current_header.replace(old, new))
                for expected, actual in zip(before, changed):
                    # 同数量的真实映射变化必须改变对应成员，不能只因计数够大而忽略。
                    self.assertNotEqual(actual, expected)
                    self.assertEqual(
                        actual,
                        expected.replace(
                            'cfg.attitude.' + old.removeprefix('&c.').removeprefix('c.'),
                            'cfg.attitude.' + new.removeprefix('&c.').removeprefix('c.'),
                        ),
                    )

    def test_minimum_mapping_guards_remain_active(self):
        for direct_count, table_count in ((8, 45), (9, 44)):
            header = '\n'.join(
                [
                    f'c.direct{i} = node.declare_parameter<bool>("direct{i}", false);'
                    for i in range(direct_count)
                ]
                + [f'{{ "table{i}", &c.table{i} }}' for i in range(table_count)]
            )
            with self.subTest(direct=direct_count, table=table_count):
                with mock.patch.object(Path, 'read_text', return_value=header):
                    with self.assertRaisesRegex(RuntimeError, 'ROS parameter mapping changed'):
                        vision_replay.parameter_mapping()

                class Probe(self.native_profile.NativeProfileContract):
                    pass

                try:
                    with (
                        mock.patch.object(Path, 'read_text', return_value=header),
                        mock.patch.object(
                            self.native_profile.shutil, 'which', return_value='/test/g++'
                        ),
                    ):
                        with self.assertRaisesRegex(
                            AssertionError, 'native adapter mapping changed'
                        ):
                            Probe.setUpClass()
                finally:
                    Probe.doClassCleanups()


if __name__ == '__main__':
    unittest.main()
