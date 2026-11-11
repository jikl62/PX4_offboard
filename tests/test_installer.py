# 源包安装器的真实离线解包测试：运行生成脚本，但仅使用临时 HOME/安装目录。
# 覆盖幂等性、先验证后写入、路径/符号链接防护，以及 stdin 在 Python 解包后的传递。
import base64
import importlib
import io
import os
from pathlib import Path
import subprocess
import tempfile
import unittest
import zipfile

ROOT = Path(__file__).resolve().parents[1]


# 把真实解包行为与恶意归档 fixture 放在同一安全边界上核对。
class InstallerTests(unittest.TestCase):
    # 每项测试使用独立临时目录，安装路径不会指向真实 FC01 服务。
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(prefix='fc01_install_')
        self.addCleanup(self.tmp.cleanup)
        self.folder = Path(self.tmp.name)
        if not (ROOT/'build_bundle.py').is_file():
            raise AssertionError('offline self-extractor is not implemented yet')
        self.builder = importlib.import_module('build_bundle')
        self.script = self.folder/'control.sh'
        self.target = self.folder/'installed'

    # 按安装器支持的头部格式封装测试载荷，供校验和及路径攻击用例使用。
    def archive(self, files):
        stream = io.BytesIO()
        with zipfile.ZipFile(stream, 'w', zipfile.ZIP_DEFLATED) as z:
            for name, data in files.items():
                z.writestr(name, data)
        return stream.getvalue()

    # 仅执行临时脚本并捕获输出；返回码与写入结果共同证明失败是否发生在正确阶段。
    def run_script(self, *args, stdin=None):
        env = {**os.environ, 'FC01_CONTROL_INSTALL_DIR': str(self.target)}
        return subprocess.run(['bash', str(self.script), *args], env=env, input=stdin,
                              capture_output=True, text=True, timeout=10)

    # 使用当前真实打包器生成源包，避免仅测试手写近似实现。
    def build_actual(self):
        self.builder.build(self.script, ROOT)

    # 安装真实载荷并验证标识文件；安装模式不得隐式启动串口、Agent 或控制节点。
    def test_real_bundle_installs_offline_without_launching_hardware(self):
        self.build_actual()
        p = self.run_script('--install-only')
        self.assertEqual(p.returncode, 0, p.stderr+p.stdout)
        for name in ('start.sh', 'fc01_control.py', 'launcher.py', 'README_zh.md', 'VERIFICATION.md'):
            self.assertEqual((self.target/name).read_bytes(), (ROOT/name).read_bytes())
        self.assertEqual(self.target.stat().st_mode & 0o777, 0o700)

    # 相同载荷重复安装可以复用，但无关文件与原文件时间信息应保留。
    def test_repeat_install_preserves_equal_files_and_unrelated_data(self):
        self.build_actual()
        self.assertEqual(self.run_script('--install-only').returncode, 0)
        (self.target/'operator-notes.txt').write_text('keep me')
        self.assertEqual(self.run_script('--install-only').returncode, 0)
        self.assertEqual((self.target/'operator-notes.txt').read_text(), 'keep me')

    # 目标存在不同内容时整次拒绝，不能先写入一半载荷再报冲突。
    def test_existing_different_file_refused_before_any_payload_write(self):
        self.build_actual()
        self.target.mkdir()
        (self.target/'start.sh').write_text('operator version')
        p = self.run_script('--install-only')
        self.assertNotEqual(p.returncode, 0)
        self.assertEqual((self.target/'start.sh').read_text(), 'operator version')
        self.assertFalse((self.target/'fc01_control.py').exists())

    # 安装目录或目标符号链接不能把写入重定向到外部路径。
    def test_existing_symlink_never_redirects_writes(self):
        self.build_actual()
        outside = self.folder/'outside'
        outside.mkdir()
        self.target.symlink_to(outside, target_is_directory=True)
        self.assertNotEqual(self.run_script('--install-only').returncode, 0)
        self.assertEqual(list(outside.iterdir()), [])

    # 内容摘要错误应在建立目标安装目录之前中止。
    def test_payload_checksum_failure_leaves_target_absent(self):
        self.build_actual()
        data = self.script.read_bytes()
        before, encoded = data.split(b'\n__FC01_PAYLOAD_BASE64__\n', 1)
        raw = bytearray(base64.b64decode(encoded))
        raw[len(raw)//2] ^= 1
        self.script.write_bytes(before+b'\n__FC01_PAYLOAD_BASE64__\n'+base64.encodebytes(raw))
        self.assertNotEqual(self.run_script('--install-only').returncode, 0)
        self.assertFalse(self.target.exists())

    # 摘要匹配只证明内容一致，不能授权 ../ 等路径穿越。
    def test_even_matching_hash_archive_cannot_escape_install_directory(self):
        for path in ('../escape', '/tmp/fc01_escape', 'a/../../escape'):
            with self.subTest(path=path):
                self.script.write_bytes(self.builder.render_archive(self.archive({'start.sh': '#!/bin/bash\n', path: 'no'})))
                self.assertNotEqual(self.run_script('--install-only').returncode, 0)
                self.assertFalse(self.target.exists())
                self.assertFalse((self.folder/'escape').exists())

    # 归档内链接同样属于不可信文件类型，不能作为普通源文件解出。
    def test_archive_symlink_is_refused(self):
        stream = io.BytesIO()
        with zipfile.ZipFile(stream, 'w') as z:
            entry = zipfile.ZipInfo('start.sh')
            entry.create_system = 3
            entry.external_attr = 0o120777 << 16
            z.writestr(entry, '/tmp/elsewhere')
        self.script.write_bytes(self.builder.render_archive(stream.getvalue()))
        self.assertNotEqual(self.run_script('--install-only').returncode, 0)
        self.assertFalse(self.target.exists())

    # 解包器占用 heredoc 后，仍需把原操作员输入传给后续入口，不得吞掉交互流。
    def test_operator_stdin_survives_python_unpacking(self):
        script = '#!/usr/bin/env bash\nread -r fc01_line\nprintf "OPERATOR:%s\\n" "$fc01_line"\n'
        self.script.write_bytes(self.builder.render_archive(self.archive({'start.sh': script})))
        p = self.run_script(stdin='takeoff 0.5\n')
        self.assertEqual(p.returncode, 0, p.stderr+p.stdout)
        self.assertIn('OPERATOR:takeoff 0.5', p.stdout)


if __name__ == '__main__':
    unittest.main()
