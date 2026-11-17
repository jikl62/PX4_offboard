# 开机待机安装器的离线合同测试：检查生成的 systemd 单元、幂等安装和有界报告。
# 临时目录和注入 runner 隔离真实服务；systemd-analyze 只校验文本，不启动控制器或 Agent。
import configparser
import importlib
import os
from pathlib import Path
import subprocess
import tempfile
import unittest
import contextlib
import io
import json
import time
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]


# 把服务文本、安装命令序列和状态显示分别验证，避免“安装成功”掩盖生命周期变化。
class AutostartTests(unittest.TestCase):
    # 按真实仓库路径加载安装器；缺失入口必须使回归明确失败。
    def module(self):
        self.assertTrue((ROOT / 'autostart.py').exists(), 'systemd installer is missing')
        return importlib.import_module('autostart')

    # 检查无 stdin、无限优雅停止期限及不发送 SIGKILL；含空格、百分号和美元符的路径必须正确转义。
    def test_generated_unit_is_valid_and_keeps_graceful_noninteractive_lifecycle(self):
        m = self.module()
        text = m.render_unit(
            'orangepi',
            Path('/home/orangepi'),
            Path('/home/orangepi/FC01 Autostart 10% $test'),
            Path('/home/orangepi/native'),
        )
        parser = configparser.ConfigParser(interpolation=None, strict=False)
        parser.read_string(text)
        s = parser['Service']
        self.assertEqual(s['User'], 'orangepi')
        self.assertEqual(s['StandardInput'], 'null')
        # External unit contract permits re-entry while the Agent is retained.
        self.assertEqual(s['KillMode'], 'process')
        self.assertEqual(s['SendSIGKILL'], 'no')
        self.assertEqual(s['TimeoutStopSec'], 'infinity')
        self.assertEqual(s['RestartSec'], '30s')
        self.assertIn('standby.sh', s['ExecStart'])
        self.assertNotIn('--flight', s['ExecStart'])
        self.assertIn('10%% $$test', s['ExecStart'])
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / 'fc01-standby.service'
            p.write_text(text)
            result = subprocess.run(
                ['/usr/bin/systemd-analyze', 'verify', str(p)], capture_output=True, text=True
            )
            self.assertEqual(result.returncode, 0, result.stderr)

    # 路径中的换行可能注入新的 ExecStart，必须在生成服务文本前拒绝。
    def test_unit_rejects_newline_in_paths(self):
        m = self.module()
        with self.assertRaises(ValueError):
            m.render_unit(
                'orangepi',
                Path('/home/orangepi'),
                Path('/tmp/bad\nExecStart=x'),
                Path('/tmp/native'),
            )

    # 已有非本工程服务属于外部资源；拒绝覆盖时连管理命令也不能执行。
    def test_foreign_unit_is_preserved_without_running_admin_commands(self):
        m = self.module()
        with tempfile.TemporaryDirectory() as d:
            dest = Path(d) / 'fc01-standby.service'
            dest.write_text('[Service]\nExecStart=/bin/true\n')
            commands = []
            with self.assertRaises(RuntimeError):
                m.install_service('new', dest, commands.append)
            self.assertEqual(commands, [])
            self.assertIn('/bin/true', dest.read_text())

    # 同一服务重复安装可确保开机启用，但不能用 restart 打断当前运行实例。
    def test_repeat_install_enables_without_restarting_active_service(self):
        m = self.module()
        with tempfile.TemporaryDirectory() as d:
            dest = Path(d) / 'fc01-standby.service'
            unit = m.render_unit(
                'orangepi', Path('/home/orangepi'), Path('/tmp/entry'), Path('/tmp/native')
            )
            dest.write_text(unit)
            commands = []

            def run(argv, **kwargs):
                commands.append(argv)
                return subprocess.CompletedProcess(argv, 0, stdout='active\n', stderr='')

            m.install_service(unit, dest, run)
            flat = [' '.join(cmd) for cmd in commands]
            self.assertTrue(any('enable --now fc01-standby.service' in x for x in flat))
            self.assertFalse(any('restart' in x or ' stop ' in x for x in flat))

    # 真实构建/解包两次并核对载荷字节；这里只执行 --install-only 和 --help，不运行飞控链。
    def test_real_offline_bundle_extracts_standby_payload_and_repeats(self):
        self.assertTrue(
            (ROOT / 'build_autostart_bundle.py').exists(), 'offline boot bundle is missing'
        )
        builder = importlib.import_module('build_autostart_bundle')
        with tempfile.TemporaryDirectory() as d:
            folder = Path(d)
            script = builder.build(folder / 'boot.sh', ROOT)
            target = folder / 'installed'
            env = {**os.environ, 'FC01_AUTOSTART_INSTALL_DIR': str(target)}
            for _ in range(2):
                p = subprocess.run(
                    ['bash', str(script), '--install-only'], env=env, capture_output=True, text=True
                )
                self.assertEqual(p.returncode, 0, p.stdout + p.stderr)
            for name in (
                'autostart.py',
                'standby.py',
                'standby.sh',
                'launcher.py',
                'fc01_control.py',
                'build_autostart_bundle.py',
                'VERIFICATION_AUTOSTART.md',
                'tests/test_standby.py',
            ):
                self.assertEqual((target / name).read_bytes(), (ROOT / name).read_bytes())
            p = subprocess.run(
                ['bash', str(target / 'autostart.sh'), '--help'], capture_output=True, text=True
            )
            self.assertEqual(p.returncode, 0, p.stderr)

    # systemd active 仅说明服务进程；100 秒前的 CONNECTED 快照必须显示过期。
    def test_status_cannot_report_old_connected_snapshot_as_current(self):
        m = self.module()
        with tempfile.TemporaryDirectory() as d:
            home = Path(d)
            folder = home / 'FC01_autostart'
            folder.mkdir()
            (folder / 'status.json').write_text(
                json.dumps(dict(status='CONNECTED', reason='old', updated_unix=time.time() - 100))
            )
            output = io.StringIO()
            with (
                patch.object(
                    m.subprocess,
                    'run',
                    return_value=subprocess.CompletedProcess([], 0, 'active\n', ''),
                ),
                contextlib.redirect_stdout(output),
            ):
                m.status(home)
            self.assertNotIn('后台：CONNECTED', output.getvalue())
            self.assertIn('STALE', output.getvalue())

    # 报告采用文件白名单且每次使用新路径，不能打包同目录无关私人文件。
    def test_report_contains_only_service_files_and_can_be_repeated(self):
        m = self.module()
        import zipfile

        with tempfile.TemporaryDirectory() as d:
            home = Path(d)
            folder = home / 'FC01_autostart'
            folder.mkdir()
            (folder / 'status.json').write_text('{}')
            (folder / 'controller.log').write_text('native trace')
            (folder / 'private-unrelated.txt').write_text('do not include')
            with patch.object(
                m.subprocess,
                'run',
                return_value=subprocess.CompletedProcess([], 0, 'service output', ''),
            ):
                first = m.report(home)
                second = m.report(home)
            self.assertNotEqual(first, second)
            with zipfile.ZipFile(first) as z:
                self.assertEqual(
                    set(z.namelist()),
                    {'status.json', 'controller.log', 'service.txt', 'journal.txt'},
                )


if __name__ == '__main__':
    unittest.main()
