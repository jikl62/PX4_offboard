"""历史服务动态对照；第三阶段仅明确允许 disk-full 的已修复清理差异，其余字节严格比较。"""

from __future__ import annotations
import json
import os
import shutil
from pathlib import Path
import subprocess
import sys
import tarfile
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
BASELINE = '5198d33f25aaaf2abc7e4b078169ae84d5c4b2c4'


class PythonServiceEquivalenceTests(unittest.TestCase):
    """比较输出、状态、命令与有序外部调用；不把数值容差用于权限和故障文本。"""

    @classmethod
    def setUpClass(cls):
        """从不可变Git对象导出基线；仅在明确指定缓存基线时复用其目录。"""
        cls.temp = tempfile.TemporaryDirectory(prefix='fc01_stage2_python_')
        cls.addClassCleanup(cls.temp.cleanup)
        cls.base = Path(cls.temp.name)
        (cls.base / '.fc01-stage2-python').write_text(BASELINE + '\n')
        existing = os.environ.get('FC01_STAGE2_BASELINE_ROOT')
        git = shutil.which('git')
        if git is None:
            raise unittest.SkipTest('历史动态对照需要Git；不影响其他普通回归')
        if existing:
            cls.baseline = Path(existing)
            head = subprocess.check_output(
                [git, '-C', str(cls.baseline), 'rev-parse', 'HEAD'], text=True
            ).strip()
            status = subprocess.check_output(
                [git, '-C', str(cls.baseline), 'status', '--porcelain=v1'], text=True
            )
            if head != BASELINE or status:
                raise AssertionError('指定的历史动态对照基线不是干净5198d33提交')
        else:
            repository = subprocess.run(
                [git, 'rev-parse', '--show-toplevel'], cwd=ROOT, capture_output=True, text=True
            )
            if repository.returncode:
                if 'not a git repository' in repository.stderr:
                    raise unittest.SkipTest('导出源码没有Git历史，无法运行历史动态对照')
                raise RuntimeError(repository.stderr)
            commit = subprocess.run(
                [git, 'rev-parse', '--verify', BASELINE + '^{commit}'],
                cwd=ROOT,
                capture_output=True,
                text=True,
            )
            if commit.returncode:
                if 'Needed a single revision' in commit.stderr:
                    raise unittest.SkipTest('当前克隆缺少5198d33历史对象')
                raise RuntimeError(commit.stderr)
            cls.baseline = cls.base / 'baseline'
            cls.baseline.mkdir()
            archive = cls.base / 'baseline.tar'
            with archive.open('wb') as output:
                subprocess.run(
                    [git, 'archive', BASELINE], cwd=ROOT, stdout=output, check=True, timeout=30
                )
            with tarfile.open(archive) as source:
                for member in source.getmembers():
                    if member.name.startswith('/') or '..' in Path(member.name).parts:
                        raise AssertionError('unsafe Git archive path')
                source.extractall(cls.baseline)

    @classmethod
    def tearDownClass(cls):
        """移除本测试自己的基线与fixture，不操作用户缓存或服务。"""
        cls.temp.cleanup()

    def compare(self, scenario):
        """隔离进程使用同一路径/输入；只允许明确列出的 probe 清理修复，其余 JSON 字节一致。"""
        values = []
        streams = []
        for root in [self.baseline, ROOT]:
            command = [
                sys.executable,
                str(ROOT / 'tests/stage2_python_trace.py'),
                str(root),
                str(self.base / 'fixture'),
                scenario,
            ]
            if scenario == 'probe' and root == ROOT:
                command.append('--probe-cleanup-fixed')
            result = subprocess.run(
                command,
                cwd=ROOT,
                capture_output=True,
                text=True,
                timeout=25,
            )
            self.assertEqual(result.returncode, 0, result.stderr)
            values.append(json.loads(result.stdout))
            streams.append(result.stdout)
        evidence = os.environ.get('FC01_STAGE2_TRACE_DIR')
        if evidence:
            target = Path(evidence)
            target.mkdir(parents=True, exist_ok=True)
            for label, value in zip(['baseline', 'current'], values):
                (target / (scenario + '-' + label + '.json')).write_text(
                    json.dumps(value, ensure_ascii=False, indent=2) + '\n'
                )
        # 比较规范JSON原字节，不能用Python的True==1或1==1.0弱相等掩盖类型差异。
        if scenario == 'probe':
            expected = json.loads(streams[0])
            disk = [row for row in expected if row['scenario'] == 'disk-full']
            self.assertEqual(len(disk), 1)
            self.assertTrue(disk[0]['known_cleanup_defect'])
            self.assertTrue(disk[0]['ros_context_alive'])
            self.assertNotIn(['destroy-node'], disk[0]['events'])
            self.assertNotIn(['shutdown'], disk[0]['events'])
            # 只允许已确认的释放缺口修复；原 OSError、订阅、证据、输出和其余四场景不能变化。
            disk[0]['events'].extend([['destroy-node'], ['shutdown']])
            disk[0]['ros_context_alive'] = False
            disk[0]['known_cleanup_defect'] = False
            expected_stream = (
                json.dumps(expected, ensure_ascii=False, sort_keys=True, allow_nan=False) + '\n'
            )
            self.assertEqual(expected_stream, streams[1], 'unexplained probe trace difference')
        else:
            self.assertEqual(streams[0], streams[1], scenario + ' baseline/current trace differs')

    def test_status_output_bytes(self):
        self.compare('render')

    def test_report_members_and_readonly_calls(self):
        self.compare('report')

    def test_service_dispatch_and_configuration(self):
        self.compare('service')

    def test_activation_transaction_and_rollback_order(self):
        self.compare('activation')

    def test_supervisor_lifetimes_and_retry_order(self):
        self.compare('standby')

    def test_console_command_and_exit_order(self):
        self.compare('launcher')

    def test_launcher_entry_configuration_and_resources(self):
        self.compare('launcher_entry')

    def test_fcctl_ack_and_bench_confirmation(self):
        self.compare('fcctl')

    def test_probe_subscriptions_and_cleanup_order(self):
        self.compare('probe')


if __name__ == '__main__':
    unittest.main()
