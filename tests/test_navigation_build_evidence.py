# ARM64 构建证据的离线回归：把编译器/依赖/ELF/运行库内容均视为可追踪输入。
# 此处以 fake runner 模拟构建步骤，不伪称已经在 ARM64 编译，也不启动服务或硬件。
"""Offline provenance/reuse regressions; no ROS nodes, services or hardware."""
import copy
import hashlib
import json
import os
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest.mock import patch

import navigation_build_evidence as evidence

ROOT = Path(__file__).resolve().parents[1]


# 建立最小源包、SDK 与运行库，并记录 configure/build/install 三个阶段的实际调用顺序。
class BuildFixture:
    # 使用临时路径和明确的 AArch64 标记，所有可变依赖都可在测试中单独替换。
    def __init__(self, folder):
        self.entry = Path(folder) / 'release'
        self.entry.mkdir()
        self.sdk = Path(folder) / 'sdk'
        self.sdk.mkdir()
        self.library = Path(folder) / 'runtime.so'
        self.library.write_bytes(b'original runtime')
        self.calls = []
        self.fail = False
        self.marker = 'FC01_RECOVERY_DIAGNOSTIC_V5_20261110'
        self.dependencies = dict(
            sdk_root=str(self.sdk),
            host_architecture='aarch64',
            tools={name: dict(path='/fake/' + name) for name in ('cmake', 'g++', 'gcc')},
            packages={
                name: dict(prefix=str(self.sdk / name))
                for name in ('px4_msgs', 'rclcpp', 'ament_cmake', 'builtin_interfaces')
            },
            trees=[dict(sha256='1' * 64)],
            environment={},
        )
        for name in evidence.REQUIRED_SOURCES:
            path = self.entry / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text('immutable payload ' + name + '\n')
        self.write_manifest()

    # 只给 fixture 自身源码生成摘要；机器相关构建目录与旧证据不作为新源输入。
    def write_manifest(self):
        files = {
            str(path.relative_to(self.entry)): evidence.digest(path)
            for path in self.entry.rglob('*')
            if path.is_file()
            and not path.is_relative_to(self.entry / '.build')
            and path.name not in ('FC01_RELEASE.json', 'ARM64_BUILD_EVIDENCE.json')
            and not path.is_relative_to(self.entry / 'install')
        }
        (self.entry / 'FC01_RELEASE.json').write_text(
            json.dumps(
                dict(source_commit='a' * 40, controller_build=self.marker, source_sha256=files)
            )
        )

    # 模拟 CMake 输出缓存与两个 AArch64 ELF，允许在 build 阶段注入失败而不写真实安装。
    def run(self, argv, **kwargs):
        self.calls.append(argv)
        if argv[1] == '-S':
            build = Path(argv[argv.index('-B') + 1])
            build.mkdir(parents=True)
            prefix = next(
                x.split('=', 1)[1] for x in argv if x.startswith('-DCMAKE_INSTALL_PREFIX=')
            )
            lines = [
                'CMAKE_CXX_COMPILER:FILEPATH=/fake/g++',
                'CMAKE_C_COMPILER:FILEPATH=/fake/gcc',
                'CMAKE_INSTALL_PREFIX:PATH=' + prefix,
            ]
            lines += [
                name + '_DIR:PATH=' + str(Path(info['prefix']) / 'share' / name / 'cmake')
                for name, info in self.dependencies['packages'].items()
            ]
            (build / 'CMakeCache.txt').write_text('\n'.join(lines) + '\n')
        if argv[1] == '--build' and self.fail:
            raise subprocess.CalledProcessError(17, argv)
        if argv[1] == '--install':
            prefix = Path(argv[2]).parent / 'install'
            for name in evidence.BINARY_NAMES:
                path = prefix / 'lib/jikl_fc01_system' / name
                path.parent.mkdir(parents=True, exist_ok=True)
                data = bytearray(64)
                data[:6] = b'\x7fELF\x02\x01'
                data[18:20] = (183).to_bytes(2, 'little')
                path.write_bytes(data + self.marker.encode())
                path.chmod(0o700)
        return subprocess.CompletedProcess(argv, 0)

    # 把单个可控运行库作为依赖指纹输入，用于验证“二进制没变但库变了”的情形。
    def libraries(self, binary):
        return {str(self.library): evidence.digest(self.library)}

    # 同时固定主机构架、依赖快照、库解析和命令执行，避免 fixture 触碰本机 ROS 安装。
    def patches(self):
        return (
            patch.object(evidence.platform, 'machine', return_value='aarch64'),
            patch.object(
                evidence,
                'dependency_snapshot',
                side_effect=lambda *_: copy.deepcopy(self.dependencies),
            ),
            patch.object(evidence, 'resolved_libraries', side_effect=self.libraries),
            patch.object(evidence.subprocess, 'run', side_effect=self.run),
        )


# 检查证据复用条件、改源重建、安装冲突和失败回滚，不能只检查 ELF 是否存在。
class NavigationBuildEvidenceTests(unittest.TestCase):
    # 每项测试恢复新的临时发布树与 patch，确保前一项的坏证据不会影响下一项。
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.fixture = BuildFixture(self.temp.name)
        for context in self.fixture.patches():
            context.start()
            self.addCleanup(context.stop)

    # 所有场景调用真实 build_release 入口，仅其外部工具由 fixture 替换。
    def build(self):
        return evidence.build_release(self.fixture.entry, self.fixture.sdk)

    # 完全相同的输入应复用一次构建；verify 不能改证据 mtime，也不能执行控制器。
    def test_matching_provenance_reuses_without_compilation_and_verify_is_read_only(self):
        first = self.build()
        self.assertEqual(len(self.fixture.calls), 3)
        evidence_path = self.fixture.entry / 'ARM64_BUILD_EVIDENCE.json'
        before = evidence_path.stat().st_mtime_ns
        self.assertEqual(self.build(), first)
        self.assertEqual(evidence.verify_build(self.fixture.entry, self.fixture.sdk), first)
        self.assertEqual(len(self.fixture.calls), 3)
        self.assertEqual(evidence_path.stat().st_mtime_ns, before)
        self.assertIs(first['controller_executed'], False)
        self.assertIs(first['agent_or_service_started'], False)
        self.assertTrue((self.fixture.entry / 'install').is_symlink())

    # 修改源后旧清单必须先拒绝；重新生成清单才可选新前缀，原安装仍保留。
    def test_modified_source_refuses_stale_manifest_and_updated_manifest_rebuilds(self):
        first = self.build()
        source = (
            self.fixture.entry
            / 'native/src/jikl_fc01_system/include/jikl_fc01_system/receive_events.hpp'
        )
        source.write_text(source.read_text() + 'new transition implementation\n')
        with self.assertRaisesRegex(evidence.BuildEvidenceError, 'source hash/path mismatch'):
            self.build()
        self.assertEqual(len(self.fixture.calls), 3)
        self.fixture.write_manifest()
        second = self.build()
        self.assertEqual(len(self.fixture.calls), 6)
        self.assertNotEqual(first['install_prefix'], second['install_prefix'])
        self.assertTrue(Path(first['install_prefix']).is_dir())
        self.assertNotEqual(first['input_sha256'], second['input_sha256'])

    # SDK 内容摘要变化即使路径相同，也必须新建安装前缀。
    def test_dependency_content_change_requires_new_build(self):
        first = self.build()
        self.fixture.dependencies['trees'][0]['sha256'] = '2' * 64
        second = self.build()
        self.assertEqual(len(self.fixture.calls), 6)
        self.assertNotEqual(first['install_prefix'], second['install_prefix'])

    # 运行库或 ELF 后续被改动时，已有完整标记不能继续授权复用。
    def test_changed_runtime_library_or_binary_cannot_be_reused(self):
        first = self.build()
        self.fixture.library.write_bytes(b'replaced runtime')
        with self.assertRaisesRegex(evidence.BuildEvidenceError, 'binary or runtime library'):
            evidence.verify_build(self.fixture.entry, self.fixture.sdk)
        second = self.build()
        self.assertNotEqual(first['install_prefix'], second['install_prefix'])
        binary = Path(second['install_prefix']) / 'lib/jikl_fc01_system/fc01_config_check'
        with binary.open('ab') as stream:
            stream.write(b'tampered')
        third = self.build()
        self.assertNotEqual(second['install_prefix'], third['install_prefix'])
        self.assertEqual(len(self.fixture.calls), 9)

    # 新增 typesupport/生成支持文件也属于安装树改变，二进制相同不等于整包完整。
    def test_changed_generated_support_file_cannot_be_reused(self):
        first = self.build()
        support = Path(first['install_prefix']) / 'share/jikl_fc01_system/generated-support'
        support.parent.mkdir(parents=True, exist_ok=True)
        support.write_text('unexpected support artifact')
        with self.assertRaisesRegex(
            evidence.BuildEvidenceError, 'generated library or support files'
        ):
            evidence.verify_build(self.fixture.entry, self.fixture.sdk)
        second = self.build()
        self.assertNotEqual(first['install_prefix'], second['install_prefix'])
        self.assertEqual(len(self.fixture.calls), 6)

    # 失败构建不可切换 install 链接或覆盖原成功证据；再次成功才发布新前缀。
    def test_interrupted_rebuild_preserves_existing_install_and_complete_evidence(self):
        first = self.build()
        saved = (self.fixture.entry / 'ARM64_BUILD_EVIDENCE.json').read_bytes()
        self.fixture.dependencies['trees'][0]['sha256'] = '3' * 64
        self.fixture.fail = True
        with self.assertRaises(subprocess.CalledProcessError):
            self.build()
        self.assertEqual((self.fixture.entry / 'install').resolve(), Path(first['install_prefix']))
        self.assertEqual((self.fixture.entry / 'ARM64_BUILD_EVIDENCE.json').read_bytes(), saved)
        self.fixture.fail = False
        new = self.build()
        self.assertNotEqual(first['install_prefix'], new['install_prefix'])
        self.assertEqual(len(self.fixture.calls), 8)

    # 不认识的普通目录和符号链接均属于外部资源，必须在编译前拒绝。
    def test_foreign_install_directory_and_symlink_are_retained(self):
        install = self.fixture.entry / 'install'
        install.mkdir()
        (install / 'do-not-delete').write_text('old')
        with self.assertRaisesRegex(evidence.BuildEvidenceError, 'existing install directory'):
            self.build()
        self.assertEqual((install / 'do-not-delete').read_text(), 'old')
        (install / 'do-not-delete').unlink()
        install.rmdir()
        install.symlink_to(self.fixture.sdk, target_is_directory=True)
        with self.assertRaisesRegex(evidence.BuildEvidenceError, 'unknown install symlink'):
            self.build()
        self.assertEqual(install.resolve(), self.fixture.sdk)
        self.assertEqual(self.fixture.calls, [])

    # 未列出的原生头文件和越界 source 路径不能通过部分摘要验证。
    def test_unlisted_source_or_traversal_is_rejected(self):
        extra = self.fixture.entry / 'native/src/jikl_fc01_system/include/extra.hpp'
        extra.write_text('new header')
        with self.assertRaisesRegex(evidence.BuildEvidenceError, 'unlisted release source'):
            self.build()
        extra.unlink()
        path = self.fixture.entry / 'FC01_RELEASE.json'
        data = json.loads(path.read_text())
        data['source_sha256']['../outside'] = hashlib.sha256(b'outside').hexdigest()
        path.write_text(json.dumps(data))
        with self.assertRaisesRegex(evidence.BuildEvidenceError, 'source hash/path mismatch'):
            self.build()
        self.assertEqual(self.fixture.calls, [])

    # 区分 ELF 架构与工程标识，不能用旧候选或 x86 可执行文件冒充当前 ARM64 制品。
    def test_wrong_elf_architecture_or_old_marker_is_rejected(self):
        path = Path(self.temp.name) / 'controller'
        data = bytearray(64)
        data[:6] = b'\x7fELF\x02\x01'
        data[18:20] = (62).to_bytes(2, 'little')
        path.write_bytes(data + self.fixture.marker.encode())
        path.chmod(0o700)
        with self.assertRaisesRegex(evidence.BuildEvidenceError, 'AArch64 ELF'):
            evidence.aarch64_binary(path, self.fixture.marker)
        data[18:20] = (183).to_bytes(2, 'little')
        for marker in (
            b'FC01_REFERENCE_PUBLICATION_V1_20261110',
            b'FC01_RECOVERY_DIAGNOSTIC_V4_20261110',
        ):
            with self.subTest(marker=marker):
                path.write_bytes(data + marker)
                with self.assertRaisesRegex(evidence.BuildEvidenceError, 'release marker'):
                    evidence.aarch64_binary(path, self.fixture.marker)

    # CMake 实际依赖目录与事先快照不一致时，校验失败并要求重新构建。
    def test_changed_cmake_dependency_cannot_pass_verification(self):
        first = self.build()
        cache = Path(first['build_dir']) / 'CMakeCache.txt'
        cache.write_text(cache.read_text().replace('/sdk/px4_msgs/', '/foreign/px4_msgs/'))
        with self.assertRaisesRegex(evidence.BuildEvidenceError, 'unexpected dependency'):
            evidence.verify_build(self.fixture.entry, self.fixture.sdk)
        self.build()
        self.assertEqual(len(self.fixture.calls), 6)


# 补充真实文件树/环境解析边界，避免只验证虚构的依赖 JSON。
class DependencyAndEnvironmentTests(unittest.TestCase):
    # 摘要跟随 SDK 链接读取真实内容，mtime 不变也能发现替换；环形链接必须明确拒绝。
    def test_dependency_hashes_follow_sdk_symlinks_and_actual_bytes(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            install = root / 'install'
            install.mkdir()
            source = root / 'source'
            source.mkdir()
            header = source / 'message.hpp'
            header.write_text('first')
            (install / 'headers').symlink_to(source, target_is_directory=True)
            old = evidence.tree_fingerprint(install)
            old_stat = header.stat()
            header.write_text('other')
            os.utime(header, ns=(old_stat.st_atime_ns, old_stat.st_mtime_ns))
            self.assertNotEqual(evidence.tree_fingerprint(install)['sha256'], old['sha256'])
            (source / 'cycle').symlink_to(install, target_is_directory=True)
            with self.assertRaisesRegex(evidence.BuildEvidenceError, 'symlink cycle'):
                evidence.tree_fingerprint(install)

    # 按显式环境、部署配置、历史默认目录解析 SDK；显式无效路径不能偷偷退回旧默认。
    def test_sdk_resolution_environment_then_config_then_existing_home(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            entry = root / 'entry'
            entry.mkdir()
            explicit = root / 'explicit'
            configured = root / 'configured'
            fallback = root / 'FC01_ARM64_PORT_20261020/FC01_R632_P0_ATTITUDE_AUDIT_CANDIDATE'
            for sdk in (explicit, configured, fallback):
                path = sdk / 'install_r632/local_setup.bash'
                path.parent.mkdir(parents=True)
                path.write_text(':\n')
            shell = 'source "$1"; fc01_resolve_sdk "$2"; printf "%s\\n" "$FC01_PROJECT_ROOT"'
            env = {**os.environ, 'HOME': str(root)}
            env.pop('FC01_PROJECT_ROOT', None)

            def resolve():
                return subprocess.run(
                    [
                        '/bin/bash',
                        '-eu',
                        '-c',
                        shell,
                        'test',
                        str(ROOT / 'navigation_environment.sh'),
                        str(entry),
                    ],
                    env=env,
                    capture_output=True,
                    text=True,
                    check=False,
                )

            self.assertEqual(resolve().stdout.strip(), str(fallback))
            config = entry / 'deployment-config.json'
            config.write_text(json.dumps({'sdk_root': str(configured)}))
            self.assertEqual(resolve().stdout.strip(), str(configured))
            env['FC01_PROJECT_ROOT'] = str(explicit)
            self.assertEqual(resolve().stdout.strip(), str(explicit))
            env['FC01_PROJECT_ROOT'] = str(root / 'missing')
            failed = resolve()
            self.assertNotEqual(failed.returncode, 0)
            self.assertIn('FC01_SDK_ENVIRONMENT_FAILED', failed.stderr)

    # x86 主机调用 ARM64 正式构建入口必须拒绝，不能伪造目标架构成功证据。
    def test_build_entry_refuses_host_without_executing_hardware(self):
        with patch.object(evidence.platform, 'machine', return_value='x86_64'):
            with self.assertRaisesRegex(evidence.BuildEvidenceError, 'native ARM64 host'):
                evidence.build_release(ROOT, Path('/missing-sdk'))


if __name__ == '__main__':
    unittest.main()
