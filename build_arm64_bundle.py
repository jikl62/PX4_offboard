#!/usr/bin/env python3
"""把已构建的 ARM64 安装树和已提交源码封装成离线发行；不编译或激活服务。"""
from __future__ import annotations

import argparse
import gzip
import hashlib
import io
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import tarfile
import tempfile

from build_navigation_bundle import names
from navigation_prebuilt_install import extract_checked, safe_name, verify_elf

ROOT = Path(__file__).resolve().parent
EXTRA = ('navigation_prebuilt_install.py', 'navigation_prebuilt_service.py', 'prebuilt_boot.sh')


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def git(*args):
    return subprocess.check_output(['git', '-C', str(ROOT), *args])


def copy_install(source, target):
    """安装文件成为普通文件，合法链接只可解引用到同一构建工作区。"""
    source = source.resolve()
    for path in sorted(source.rglob('*')):
        rel = path.relative_to(source)
        if '__pycache__' in rel.parts or path.suffix == '.pyc':
            continue
        if path.is_symlink() and path.is_dir():
            raise ValueError('directory symlink is not a portable install item: ' + str(path))
        if path.is_dir():
            continue
        if not path.is_file() or not path.resolve().is_relative_to(source.parent):
            raise ValueError('install item is not a bounded ordinary file: ' + str(path))
        dest = target / rel
        dest.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(path, dest)
        dest.chmod(0o755 if os.access(path, os.X_OK) else 0o644)
        verify_elf(dest)


def validate_evidence(entry, evidence, candidate):
    """构建证据必须精确覆盖实际CMake输入与全部可搬迁安装产物。"""
    if (evidence.get('control_candidate_commit') != candidate
            or evidence.get('architecture') != 'aarch64'
            or evidence.get('ubuntu') != '22.04' or evidence.get('ros_distro') != 'humble'):
        raise ValueError('build evidence identity mismatch')
    sources = {p.relative_to(entry).as_posix()
               for p in (entry / 'native/src/jikl_fc01_system').rglob('*') if p.is_file()}
    sources.update('native/tools/' + name for name in
                   ('vision_odometry_adapter.py', 'velocity_command_diagnostic.py',
                    'vision_state_validation.py', 'vision_resource_budget.py'))
    installed = {p.relative_to(entry / 'install').as_posix()
                 for p in (entry / 'install').rglob('*') if p.is_file()}
    if not installed or not {'lib/jikl_fc01_system/fc01_system_node',
                             'lib/jikl_fc01_system/fc01_config_check'} <= installed:
        raise ValueError('build evidence lacks controller/checker')
    for key, expected, prefix in (('native_source_sha256', sources, entry),
                                  ('install_sha256', installed, entry / 'install')):
        values = evidence.get(key)
        if not isinstance(values, dict) or set(values) != expected:
            raise ValueError('build evidence coverage differs: ' + key)
        for name, digest in values.items():
            if (not safe_name(name) or not re.fullmatch(r'[0-9a-f]{64}', str(digest))
                    or sha(prefix / name) != digest):
                raise ValueError('build evidence bytes differ: ' + name)


def build(args):
    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.-]{0,95}', args.version):
        raise ValueError('invalid release version')
    commit = git('rev-parse', 'HEAD').decode().strip()
    candidate = git('rev-parse', args.control_candidate + '^{commit}').decode().strip()
    # 只归档提交的字节；用户其他未跟踪文件和历史证据不进入运行载荷。
    with tempfile.TemporaryDirectory(prefix='fc01-arm64-release-') as temporary:
        stage = Path(temporary)
        source = stage / 'source'
        source.mkdir()
        with tarfile.open(fileobj=io.BytesIO(git('archive', commit))) as archive:
            archive.extractall(source)
        top = 'FC01_OrangePi_' + args.version + '_arm64'
        entry = stage / top
        entry.mkdir()
        selected = sorted(set(names(source)) | set(EXTRA))
        for name in selected:
            original = source / name
            if not original.is_file():
                raise ValueError('missing committed runtime input: ' + name)
            if name.startswith('native/src/') or name.startswith('native/tools/'):
                baseline = git('show', candidate + ':' + name)
                if baseline != original.read_bytes():
                    raise ValueError('control candidate bytes changed: ' + name)
            dest = entry / name
            dest.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(original, dest)
        copy_install(args.install_tree, entry / 'install')
        evidence = json.loads(args.build_evidence.read_text())
        validate_evidence(entry, evidence, candidate)
        (entry / 'ARM64_BUILD.json').write_text(json.dumps(evidence, ensure_ascii=False, indent=2) + '\n')
        manifest = dict(version=args.version, source_commit=commit,
                        control_candidate_commit=candidate, architecture='aarch64',
                        ubuntu='22.04', ros_distro='humble',
                        controller_relative='install/lib/jikl_fc01_system/fc01_system_node',
                        system_dependencies=['ROS 2 Humble rclcpp/rclpy/rmw_fastrtps_cpp',
                                             'python3-yaml', 'psmisc', 'systemd', 'sudo',
                                             'Ubuntu 22.04 ARM64 glibc/libstdc++'],
                        payload_sha256={p.relative_to(entry).as_posix(): sha(p)
                                        for p in sorted(entry.rglob('*')) if p.is_file()})
        (entry / 'ARM64_RELEASE.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + '\n')
        args.output.mkdir(parents=True, exist_ok=True)
        output = args.output / (top + '.tar.gz')
        # 稳定元数据，避免打包用户名/临时目录和时间影响交付身份。
        with output.open('wb') as raw, gzip.GzipFile(fileobj=raw, mode='wb', mtime=0, filename='') as compressed:
            with tarfile.open(fileobj=compressed, mode='w') as archive:
                for p in sorted(entry.rglob('*')):
                    if not p.is_file():
                        continue
                    info = tarfile.TarInfo(top + '/' + p.relative_to(entry).as_posix())
                    info.size = p.stat().st_size
                    info.mode = 0o755 if os.access(p, os.X_OK) else 0o644
                    with p.open('rb') as stream:
                        archive.addfile(info, stream)
        archive_sha = sha(output)
        with tempfile.TemporaryDirectory(prefix='fc01-arm64-readback-') as checkdir:
            _, readback = extract_checked(output, archive_sha, Path(checkdir))
            if readback != manifest:
                raise ValueError('archive manifest readback differs')
        installer_source = (source / 'navigation_prebuilt_install.py').read_text()
        pin_target = "parser.add_argument('--expected-sha256')"
        if installer_source.count(pin_target) != 1:
            raise ValueError('standalone installer SHA pin contract changed')
        installer_source = installer_source.replace(
            pin_target,
            "parser.add_argument('--expected-sha256', default=" + repr(archive_sha)
            + ', choices=[' + repr(archive_sha) + '])')
        script = '#!/usr/bin/env bash\n# 本安装器只接受相配的发行包摘要。以普通用户运行。\nset -euo pipefail\nexport PYTHONDONTWRITEBYTECODE=1\n/usr/bin/python3 - "$@" <<\'FC01_PREBUILT_INSTALL_PY\'\n' + installer_source + '\nFC01_PREBUILT_INSTALL_PY\n'
        install_script = args.output / 'install_fc01.sh'
        install_script.write_text(script)
        install_script.chmod(0o755)
        Path(str(output) + '.sha256').write_text(archive_sha + '  ' + output.name + '\n')
        result = dict(archive=str(output), bytes=output.stat().st_size, sha256=archive_sha,
                      installer=str(install_script), installer_sha256=sha(install_script),
                      source_commit=commit, control_candidate_commit=candidate,
                      files=len(manifest['payload_sha256']) + 1)
        (args.output / 'DELIVERY.json').write_text(json.dumps(result, indent=2) + '\n')
        print(json.dumps(result, ensure_ascii=False, indent=2))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--install-tree', required=True, type=Path)
    parser.add_argument('--build-evidence', required=True, type=Path)
    parser.add_argument('--version', required=True)
    parser.add_argument('--control-candidate', default='adc55d0b7c67e80bf9d77b1082208c898f95b756')
    parser.add_argument('--output', type=Path, default=ROOT / 'dist/arm64')
    build(parser.parse_args())


if __name__ == '__main__':
    main()
