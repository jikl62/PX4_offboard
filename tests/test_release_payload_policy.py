"""运行源码包边界：开发历史留在仓库，运行依赖按当前字节交付，解包安全预算不放宽。

所有安装器只执行 --extract-only，目标、HOME 和日志均在临时目录；不运行部署或控制器。
期望文件按实际运行依赖列出，不从被测白名单生成，避免遗漏依赖时测试也跟着遗漏。
"""

import ast
import base64
import hashlib
import importlib
import io
import json
import os
from pathlib import Path
import stat
import subprocess
import tempfile
import unittest
import zipfile


bundle = importlib.import_module('build_navigation_bundle')
ROOT = Path(__file__).resolve().parents[1]


class ReleasePayloadPolicyTests(unittest.TestCase):
    """独立核对发行范围、实际依赖闭包，以及 builder/extractor 两层拒绝。"""

    def fixture(self, directory):
        """只建本例拥有的微型运行树；所有内容都由测试写入，绝不指向真实服务。"""
        source = Path(directory) / 'source'
        source.mkdir()
        for name, data in {
            'navigation.sh': b'#!/bin/bash\nexit 0\n',
            'navigation_deploy.sh': b'#!/bin/bash\nexit 97\n',
            'navigation_service.py': b'CURRENT_LOCAL_SERVICE_EDIT=True\n',
            'navigation_settings.json': b'{"target_height_m":1.5}\n',
            'native/src/jikl_fc01_system/src/fc01_system_node.cpp': b'current node\n',
            'native/src/jikl_fc01_system/include/jikl_fc01_system/local_new.hpp': b'new header\n',
            'native/tools/vision_odometry_adapter.py': b'current adapter\n',
            'native/tools/velocity_command_diagnostic.py': b'current diagnostic\n',
            'native/tools/vision_state_validation.py': b'current validation\n',
            'native/tools/vision_resource_budget.py': b'current budgets\n',
            'native/config/flight_test.yaml': b'target: 1.5\n',
        }.items():
            path = source / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(data)
        return source

    def malicious_archive(self, directory, entries):
        """只重算外层 SHA；恶意成员仍须被真实 HEADER 独立拒绝，不能依赖 builder 过滤。"""
        stream = io.BytesIO()
        with zipfile.ZipFile(stream, 'w', zipfile.ZIP_DEFLATED) as archive:
            for name, data in entries:
                info = zipfile.ZipInfo(name)
                info.create_system = 3
                info.external_attr = (stat.S_IFREG | 0o600) << 16
                info.compress_type = zipfile.ZIP_DEFLATED
                archive.writestr(info, data)
        payload = stream.getvalue()
        header = (
            bundle.HEADER.replace('__ARCHIVE_SHA256__', hashlib.sha256(payload).hexdigest())
            .replace('__DIRECTORY_NAME__', 'isolated-extraction')
            .replace('__RELEASE_NAME__', 'FC01_OrangePi_Navigation_20261112_release')
            .replace('__RELEASE_VERSION__', '20261112.1')
            .replace('__CONTROLLER_BUILD__', 'FC01_RECOVERY_DIAGNOSTIC_V5_20261110')
        )
        installer = Path(directory) / 'unsafe.sh'
        installer.write_bytes(header.encode() + base64.encodebytes(payload))
        return installer

    def test_development_history_archives_caches_and_experiments_stay_outside_runtime(self):
        """大历史 ZIP 不应挤占运行预算，排除只影响载荷，不删除源树任何文件。"""
        with tempfile.TemporaryDirectory() as directory:
            source = self.fixture(directory)
            excluded = (
                'docs/readability/overnight_20261112/evidence.zip',
                'docs/evidence/old_trace.json',
                'docs/old_flight_review.md',
                'native/tests/test_old.cpp',
                'native/tests/ros_receive_ab/experiment.py',
                'native/tests/historical_failure/replay.cpp',
                'native/scripts/experimental_start.sh',
                'native/build/object.o',
                'native/install/lib/old.so',
                'native/tools/__pycache__/vision.pyc',
                'native/tools/old_dump.zip',
                'native/src/jikl_fc01_system/build/old.o',
                'native/config/flight_test.yaml.bak',
                'dist/old-installer.sh',
                'build/temporary.cpp',
                'developer_new_helper.py',
                'HOST_VERIFICATION.json',
                'progress.md',
            )
            for name in excluded:
                path = source / name
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_bytes(b'preserve development history\n')
            # 大于每项 1 MiB 的历史证据仍可保留在仓库；它不属于运行安装载荷。
            evidence = source / excluded[0]
            evidence.write_bytes(b'x' * (1024 * 1024 + 1))
            before = {name: (source / name).read_bytes() for name in excluded}
            selected = set(bundle.names(source))
            self.assertTrue(set(excluded).isdisjoint(selected), sorted(selected & set(excluded)))
            manifest = bundle.make_manifest(source)
            self.assertTrue(set(excluded).isdisjoint(manifest['payload_sha256']))
            self.assertEqual(
                manifest['payload_sha256']['navigation_service.py'],
                hashlib.sha256(b'CURRENT_LOCAL_SERVICE_EDIT=True\n').hexdigest(),
            )
            for name, data in before.items():
                self.assertEqual((source / name).read_bytes(), data, name)

    def test_actual_runtime_import_dependencies_vision_and_messages_are_complete(self):
        """运行树必须自带本项目导入依赖，不能为了容量仅保留控制器而遗漏视觉消息/工具。"""
        selected = set(bundle.names(ROOT))
        required = {
            'navigation.sh',
            'navigation_boot.sh',
            'navigation_deploy.sh',
            'navigation_environment.sh',
            'navigation_build_fresh.sh',
            'navigation_restore.sh',
            'navigation.py',
            'navigation_install.py',
            'navigation_deploy.py',
            'navigation_service.py',
            'navigation_status.py',
            'navigation_probe.py',
            'navigation_collect.py',
            'navigation_report.py',
            'navigation_restore.py',
            'navigation_build_evidence.py',
            'feedback_age_probe.py',
            'launcher.py',
            'standby.py',
            'fc01_control.py',
            'uart_preflight.py',
            'vision_settings.py',
            'vision_config.py',
            'navigation_settings.json',
            'vision_settings.json',
            'native/config/diagnostic.yaml',
            'native/config/flight_test.yaml',
            'native/tools/vision_odometry_adapter.py',
            'native/tools/velocity_command_diagnostic.py',
            'native/tools/vision_state_validation.py',
            'native/tools/vision_resource_budget.py',
        }
        package = ROOT / 'native/src/jikl_fc01_system'
        required.update(
            path.relative_to(ROOT).as_posix()
            for path in package.rglob('*')
            if path.is_file() and path.suffix in ('.hpp', '.cpp', '.msg')
        )
        required.update(
            {
                'native/src/jikl_fc01_system/CMakeLists.txt',
                'native/src/jikl_fc01_system/package.xml',
            }
        )
        self.assertTrue(required <= selected, sorted(required - selected))
        self.assertEqual(len(list((package / 'msg').glob('*.msg'))), 5)
        # 跟踪运行入口的项目内 import；外部 ROS/yaml 等依赖另由环境核验，不假装随包提供。
        for name in selected:
            if not name.endswith('.py'):
                continue
            path = ROOT / name
            for statement in ast.walk(ast.parse(path.read_bytes(), filename=name)):
                modules = (
                    [alias.name for alias in statement.names]
                    if isinstance(statement, ast.Import)
                    else [statement.module] if isinstance(statement, ast.ImportFrom) else []
                )
                for module in modules:
                    if not module:
                        continue
                    module_file = module.split('.')[0] + '.py'
                    for candidate in (path.parent / module_file, ROOT / module_file):
                        if candidate.is_file():
                            self.assertIn(candidate.relative_to(ROOT).as_posix(), selected, name)

    def test_builder_keeps_file_count_total_and_member_size_limits(self):
        """分别构造超过真实安全上限的运行内容；不修改 MAX_*、不把拒绝改为通过。"""
        self.assertEqual(bundle.MAX_FILES, 512)
        self.assertEqual(bundle.MAX_BYTES, 8 * 1024 * 1024)
        for kind in ('count', 'total', 'member'):
            with self.subTest(limit=kind), tempfile.TemporaryDirectory() as directory:
                source = self.fixture(directory)
                extra = source / 'native/src/jikl_fc01_system/include/jikl_fc01_system'
                if kind == 'count':
                    for index in range(513):
                        (extra / f'item{index}.hpp').write_bytes(b'// source\n')
                elif kind == 'total':
                    for index in range(9):
                        (extra / f'item{index}.hpp').write_bytes(b'x' * (1024 * 1024))
                else:
                    (extra / 'oversized.hpp').write_bytes(b'x' * (1024 * 1024 + 1))
                output = Path(directory) / 'rejected.sh'
                with self.assertRaisesRegex(RuntimeError, 'bounds'):
                    bundle.build(output, source)
                self.assertFalse(output.exists())
                self.assertFalse(Path(str(output) + '.sha256').exists())

    def test_extractor_independently_keeps_all_three_capacity_limits(self):
        """直接造归档绕开 builder，仍须在创建安装目标前因原容量限制失败。"""
        for kind, entries, message in (
            ('count', [(f'item{i}.txt', b'x') for i in range(513)], '文件数量超限'),
            ('member', [('large.hpp', b'x' * (1024 * 1024 + 1))], '体积超限'),
            ('total', [(f'part{i}.hpp', b'x' * (1024 * 1024)) for i in range(9)], '体积超限'),
        ):
            with self.subTest(limit=kind), tempfile.TemporaryDirectory() as directory:
                home = Path(directory) / 'home'
                home.mkdir()
                installer = self.malicious_archive(directory, entries)
                env = {**os.environ, 'HOME': str(home)}
                env.pop('FC01_NAVIGATION_INSTALL_DIR', None)
                result = subprocess.run(
                    ['bash', str(installer), '--extract-only'],
                    env=env,
                    capture_output=True,
                    text=True,
                )
                self.assertNotEqual(result.returncode, 0)
                self.assertIn(message, result.stderr)
                self.assertFalse((home / 'isolated-extraction').exists())
                self.assertFalse(list(home.glob('.isolated-extraction.unpack-*')))

    def test_manifest_cannot_omit_current_native_source_hash(self):
        """payload 摘要正确也不能省掉原生源码身份，过期清单不会自动刷新为成功。"""
        with tempfile.TemporaryDirectory() as directory:
            source = self.fixture(directory)
            manifest = bundle.make_manifest(source)
            missing = 'native/src/jikl_fc01_system/include/jikl_fc01_system/local_new.hpp'
            del manifest['source_sha256'][missing]
            (source / bundle.MANIFEST).write_text(json.dumps(manifest))
            output = Path(directory) / 'rejected.sh'
            with self.assertRaisesRegex(ValueError, 'source hashes'):
                bundle.build(output, source)
            self.assertFalse(output.exists())
            self.assertEqual(json.loads((source / bundle.MANIFEST).read_text()), manifest)

    def test_old_release_manifest_is_preserved_and_not_silently_refreshed(self):
        """旧身份即使所有内容 SHA 都匹配，也不能被静默解释成本次正式发行清单。"""
        with tempfile.TemporaryDirectory() as directory:
            source = self.fixture(directory)
            manifest = bundle.make_manifest(source)
            manifest.update(
                release_name='FC01_OrangePi_Navigation_20261110_recovery_diagnostic_v5',
                version='20261110.7',
            )
            old_bytes = (json.dumps(manifest, indent=2, sort_keys=True) + '\n').encode()
            (source / bundle.MANIFEST).write_bytes(old_bytes)
            output = Path(directory) / 'rejected.sh'
            with self.assertRaisesRegex(ValueError, 'identity differs'):
                bundle.build(output, source)
            self.assertFalse(output.exists())
            self.assertEqual((source / bundle.MANIFEST).read_bytes(), old_bytes)

    def test_release_manifest_itself_still_obeys_per_member_budget(self):
        """运行源码再小，附加的发布元数据也必须计入每项预算，不能绕过解包器上限。"""
        with tempfile.TemporaryDirectory() as directory:
            source = self.fixture(directory)
            manifest = bundle.make_manifest(source)
            manifest['extra_review_metadata'] = 'x' * (1024 * 1024)
            (source / bundle.MANIFEST).write_text(json.dumps(manifest))
            output = Path(directory) / 'rejected.sh'
            with self.assertRaisesRegex(RuntimeError, 'bounds'):
                bundle.build(output, source)
            self.assertFalse(output.exists())

    def test_selected_source_symlink_is_rejected_without_reading_as_regular_source(self):
        """运行白名单中的链接不能伪装源码；外部文件仍由原所有者保留。"""
        for name in (
            'navigation_service.py',
            'native/src/jikl_fc01_system/include/jikl_fc01_system/local_new.hpp',
        ):
            with self.subTest(path=name), tempfile.TemporaryDirectory() as directory:
                source = self.fixture(directory)
                outside = Path(directory) / 'outside.txt'
                outside.write_bytes(b'owned outside source tree\n')
                link = source / name
                link.unlink()
                link.symlink_to(outside)
                with self.assertRaisesRegex(ValueError, 'symlink'):
                    bundle.build(Path(directory) / 'rejected.sh', source)
                self.assertEqual(outside.read_bytes(), b'owned outside source tree\n')


if __name__ == '__main__':
    unittest.main()
