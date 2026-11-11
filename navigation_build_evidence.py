#!/usr/bin/env python3
# 原生ARM64构建来源证明：源码/依赖/编译器/实际安装树与动态库必须共同对应，才允许复用。
# 不运行控制器、Agent、探针或服务；x86测试fixture不能产生真实ARM64验收结论。
"""Native ARM64 build provenance and read-only verification; never starts ROS nodes."""
from __future__ import annotations

import argparse
from contextlib import contextmanager
from datetime import datetime, timezone
import fcntl
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import platform
import re
import shutil
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET

# schema2绑定完整安装树和构建输入；升级前的证据不按兼容假设复用。
SCHEMA = 2
BINARY_NAMES = ('fc01_system_node', 'fc01_config_check')
# 这些源文件必须出现在清单；另外递归拒绝native源码/配置中新出现的未列文件。
REQUIRED_SOURCES = (
    'navigation_build_fresh.sh', 'navigation_environment.sh', 'navigation_build_evidence.py',
    'native/config/flight_test.yaml', 'native/config/diagnostic.yaml',
    'native/src/jikl_fc01_system/CMakeLists.txt',
    'native/src/jikl_fc01_system/src/fc01_system_node.cpp',
    'native/src/jikl_fc01_system/src/fc01_config_check.cpp',
    'native/src/jikl_fc01_system/include/jikl_fc01_system/receive_events.hpp',
    'native/src/jikl_fc01_system/include/jikl_fc01_system/vertical_feedback_guard.hpp',
    'native/src/jikl_fc01_system/include/jikl_fc01_system/attitude_altitude_core.hpp',
    'native/src/jikl_fc01_system/include/jikl_fc01_system/rc_navigation.hpp',
)
ENVIRONMENT_KEYS = ('ROS_DISTRO', 'AMENT_PREFIX_PATH', 'CMAKE_PREFIX_PATH', 'LD_LIBRARY_PATH',
                    'PYTHONPATH', 'CFLAGS', 'CXXFLAGS', 'CPPFLAGS', 'LDFLAGS',
                    'CPATH', 'CPLUS_INCLUDE_PATH', 'C_INCLUDE_PATH', 'LIBRARY_PATH',
                    'COMPILER_PATH', 'PKG_CONFIG_PATH')


# 构建身份不可信的显式拒绝异常，不自动绕过或降级校验。
class BuildEvidenceError(RuntimeError):
    pass


# 分块摘要真实文件字节，不依赖文件mtime；避免一次把大依赖库读入内存。
def digest(path):
    value = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024*1024), b''):
            value.update(block)
    return value.hexdigest()


# 稳定排序与紧凑JSON序列化使同一结构有相同摘要，供输入索引和证据校验。
def object_digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':')).encode()).hexdigest()


# 无交互运行只读工具查询，非零保留stdout/stderr后拒绝，不能静默选另一工具。
def command(argv):
    result = subprocess.run(list(map(str, argv)), stdin=subprocess.DEVNULL,
                            capture_output=True, text=True, check=False)
    if result.returncode:
        raise BuildEvidenceError('command failed: '+str(argv)+'\n'+result.stdout+result.stderr)
    return result.stdout.strip()


# 正式入口仅接受本机aarch64/arm64，禁止把host或交叉编译文件冒充目标机原生证据。
def require_arm64():
    if platform.machine() not in ('aarch64', 'arm64'):
        raise BuildEvidenceError('native ARM64 host required; no cross-built or host binary accepted')


# 验证提交标识、源码路径/SHA与编译标记，拒绝符号链接/路径穿越和不完整源树。
def release_sources(entry):
    entry = Path(entry).resolve()
    manifest_path = entry/'FC01_RELEASE.json'
    try:
        release = json.loads(manifest_path.read_text())
    except (OSError, ValueError) as exc:
        raise BuildEvidenceError('missing or invalid FC01_RELEASE.json: '+str(exc)) from exc
    files = release.get('source_sha256')
    if (not re.fullmatch(r'[0-9a-f]{40}', str(release.get('source_commit', ''))) or
            not isinstance(files, dict) or any(name not in files for name in REQUIRED_SOURCES)):
        raise BuildEvidenceError('incomplete release source manifest')
    marker = release.get('controller_build')
    if not isinstance(marker, str) or not re.fullmatch(r'FC01_[A-Z0-9_]+', marker):
        raise BuildEvidenceError('missing or invalid release controller_build')
    for name, expected in files.items():
        relative = PurePosixPath(name)
        path = entry/name
        if (relative.is_absolute() or '..' in relative.parts or not relative.parts or
                path.is_symlink() or not path.resolve().is_relative_to(entry) or not path.is_file() or
                not re.fullmatch(r'[0-9a-f]{64}', str(expected)) or digest(path) != expected):
            raise BuildEvidenceError('release source hash/path mismatch: '+name)
    # An unlisted translation unit/header could alter CMake's result despite
    # every listed file still matching. Refuse a partially described source tree.
    for root in (entry/'native/src', entry/'native/config'):
        for path in root.rglob('*'):
            if path.is_symlink():
                raise BuildEvidenceError('symlink in immutable source tree: '+str(path))
            if path.is_file() and str(path.relative_to(entry)) not in files:
                raise BuildEvidenceError('unlisted release source: '+str(path.relative_to(entry)))
    return dict(source_commit=release['source_commit'], controller_build=marker,
                release_manifest_sha256=digest(manifest_path),
                source_sha256=object_digest(files))


# 递归摘要依赖实际目标字节与解析路径；跟随合法SDK链接、拒绝环与断链，排除Python缓存。
def tree_fingerprint(root):
    """Hash actual dependency bytes, including symlinked SDK install trees."""
    root = Path(root)
    if not root.is_dir():
        raise BuildEvidenceError('dependency directory is missing: '+str(root))
    value = hashlib.sha256(); count = total = 0

    # ancestors只记录当前递归链，防止目录链接成环；每个相对路径及真实路径都纳入摘要。
    def walk(directory, ancestors):
        nonlocal count, total
        real = directory.resolve()
        if real in ancestors:
            raise BuildEvidenceError('dependency directory symlink cycle: '+str(directory))
        for child in sorted(directory.iterdir(), key=lambda p: p.name):
            if child.name == '__pycache__' or child.suffix in ('.pyc', '.pyo'):
                continue
            relative = str(child.relative_to(root))
            if child.is_dir():
                value.update(('D\0'+relative+'\0'+str(child.resolve())+'\0').encode())
                walk(child, ancestors | {real})
            elif child.is_file():
                size = child.stat().st_size
                value.update(('F\0'+relative+'\0'+str(child.resolve())+'\0'+digest(child)+'\0').encode())
                count += 1; total += size
            elif child.is_symlink():
                raise BuildEvidenceError('broken dependency symlink: '+str(child))
    walk(root, set())
    return dict(path=str(root.resolve()), sha256=value.hexdigest(), files=count, bytes=total)


# 从PATH找到真实工具后解析symlink、摘要可执行文件及版本输出，而非只记录名称。
def executable_fingerprint(name, *version_args):
    located = shutil.which(name)
    if not located:
        raise BuildEvidenceError('required installed tool is missing: '+name)
    path = Path(located).resolve()
    return dict(path=str(path), sha256=digest(path), version=command([path, *version_args]))


# 强制Humble且px4_msgs来自所选SDK；收集工具、包版本、头/库/生成器树与影响编译的环境。
def dependency_snapshot(entry, sdk):
    sdk = Path(sdk).expanduser().resolve()
    if os.environ.get('ROS_DISTRO') != 'humble':
        raise BuildEvidenceError('ROS 2 Humble environment is not loaded')
    sdk_install = sdk/'install_r632'
    if not (sdk_install/'local_setup.bash').is_file():
        raise BuildEvidenceError('SDK install_r632/local_setup.bash is missing: '+str(sdk))
    px4_prefix = Path(command(['ros2', 'pkg', 'prefix', 'px4_msgs'])).resolve()
    if not px4_prefix.is_relative_to(sdk_install.resolve()):
        raise BuildEvidenceError('px4_msgs resolves outside the selected SDK: '+str(px4_prefix))
    ros_prefix = Path('/opt/ros/humble')
    tools = {name: executable_fingerprint(name, '--version') for name in ('cmake', 'g++', 'gcc', 'make', 'ld', 'as')}
    tools['python3'] = executable_fingerprint('/usr/bin/python3', '--version')
    tools['ros2'] = dict(path=str(Path(shutil.which('ros2')).resolve()),
                         sha256=digest(shutil.which('ros2')))
    # Hash installed ROS/PX4 headers, CMake generators, Python support, shared
    # libraries, GCC private headers/libraries and system headers by content.
    # Runtime system libraries actually selected by ldd are checked separately.
    roots = [ros_prefix, sdk_install, Path('/usr/include')]
    if Path('/usr/local/include').is_dir():
        roots.append(Path('/usr/local/include'))
    for variable in ('CPATH', 'CPLUS_INCLUDE_PATH', 'C_INCLUDE_PATH'):
        for part in os.environ.get(variable, '').split(os.pathsep):
            if part and Path(part).is_dir() and Path(part).resolve() not in roots:
                roots.append(Path(part).resolve())
    cmake_version = re.search(r'cmake version (\d+\.\d+)', tools['cmake']['version'])
    if not cmake_version:
        raise BuildEvidenceError('cannot identify installed CMake module version')
    roots.append(Path(tools['cmake']['path']).parent.parent/'share'/('cmake-'+cmake_version[1]))
    gcc_private = Path(command([tools['g++']['path'], '-print-libgcc-file-name'])).resolve().parent
    if gcc_private not in roots:
        roots.append(gcc_private)
    packages = {}
    for name in ('px4_msgs', 'rclcpp', 'ament_cmake', 'builtin_interfaces', 'rosidl_default_generators'):
        prefix = Path(command(['ros2', 'pkg', 'prefix', name])).resolve()
        expected = px4_prefix if name == 'px4_msgs' else ros_prefix
        if name != 'px4_msgs' and prefix != expected:
            raise BuildEvidenceError('ROS dependency resolves outside Humble: '+name+' '+str(prefix))
        package = prefix/'share'/name/'package.xml'
        packages[name] = dict(prefix=str(prefix), version=ET.fromstring(package.read_text()).findtext('version'),
                              package_sha256=digest(package))
    return dict(sdk_root=str(sdk), host_architecture=platform.machine(), ros_distro='humble',
                tools=tools, packages=packages,
                trees=[tree_fingerprint(path) for path in roots],
                environment={key: os.environ.get(key, '') for key in ENVIRONMENT_KEYS})


# 构建键同时包含不可变release源和当前依赖；任何一侧变化都不能复用旧产物。
def build_inputs(entry, sdk):
    return dict(release=release_sources(entry), dependencies=dependency_snapshot(entry, sdk))


# 验证ELF64小端e_machine=183、可执行权限与本版控制marker，不启动被检查程序。
def aarch64_binary(path, marker=None):
    path = Path(path)
    data = path.read_bytes()
    if (len(data) < 64 or data[:6] != b'\x7fELF\x02\x01' or
            int.from_bytes(data[18:20], 'little') != 183 or not os.access(path, os.X_OK)):
        raise BuildEvidenceError('not an executable little-endian AArch64 ELF: '+str(path))
    if marker is not None and marker.encode() not in data:
        raise BuildEvidenceError('compiled release marker is missing: '+str(path))
    return dict(path=str(path.resolve()), elf_machine='AArch64', sha256=hashlib.sha256(data).hexdigest(),
                safety_marker=marker)


# 用ldd确认真实解析库路径与内容，拒绝not found；ASLR地址不纳入摘要。
def resolved_libraries(binary):
    result = subprocess.run(['ldd', str(binary)], stdin=subprocess.DEVNULL,
                            capture_output=True, text=True, check=False)
    report = result.stdout+result.stderr
    if result.returncode or 'not found' in report:
        raise BuildEvidenceError('unresolved runtime dependencies: '+str(binary)+'\n'+report)
    libraries = {}
    for line in report.splitlines():
        match = re.search(r'(?:=>\s+)?(/\S+)\s+\(', line)
        if match:
            path = Path(match.group(1)).resolve()
            libraries[str(path)] = digest(path)
    if not libraries:
        raise BuildEvidenceError('no dynamic runtime dependencies resolved: '+str(binary))
    # ASLR addresses are deliberately excluded; resolved path AND bytes matter.
    return libraries


# 核对CMake实际选中的compiler和消息/ROS包，避免配置记录与预先指纹不一致。
def cmake_evidence(build, dependencies):
    cache = Path(build)/'CMakeCache.txt'
    parsed = {}
    for line in cache.read_text().splitlines():
        if re.match(r'(?:CMAKE_CXX_COMPILER|CMAKE_C_COMPILER|CMAKE_HOME_DIRECTORY|CMAKE_INSTALL_PREFIX|px4_msgs_DIR|rclcpp_DIR|ament_cmake_DIR|builtin_interfaces_DIR):[^=]+=', line):
            key, value = line.split('=', 1); parsed[key.split(':')[0]] = value
    for name in ('px4_msgs', 'rclcpp', 'ament_cmake', 'builtin_interfaces'):
        expected = Path(dependencies['packages'][name]['prefix'])/'share'/name/'cmake'
        if Path(parsed.get(name+'_DIR', '/missing')).resolve() != expected.resolve():
            raise BuildEvidenceError('CMake selected an unexpected dependency: '+name)
    for key, tool in (('CMAKE_CXX_COMPILER', 'g++'), ('CMAKE_C_COMPILER', 'gcc')):
        if Path(parsed.get(key, '/missing')).resolve() != Path(dependencies['tools'][tool]['path']):
            raise BuildEvidenceError('CMake compiler differs from fingerprinted compiler: '+key)
    return dict(cache_sha256=digest(cache), selected=parsed)


# 控制器及配置检查器各自记录ELF与实际动态库，控制器额外核验release marker。
def binary_evidence(prefix, marker):
    result = {}
    for name in BINARY_NAMES:
        binary = Path(prefix)/'lib/jikl_fc01_system'/name
        row = aarch64_binary(binary, marker if name == 'fc01_system_node' else None)
        row['resolved_libraries'] = resolved_libraries(binary)
        result[name] = row
    return result


# 完整复用检查：输入、安装链接归属、二进制/库、全安装树与CMake选择全部匹配才返回证据。
def _verify(entry, sdk, inputs=None):
    entry = Path(entry).resolve()
    inputs = build_inputs(entry, sdk) if inputs is None else inputs
    evidence = json.loads((entry/'ARM64_BUILD_EVIDENCE.json').read_text())
    install = entry/'install'
    if (evidence.get('schema') != SCHEMA or evidence.get('entry') != str(entry) or
            evidence.get('inputs') != inputs or evidence.get('input_sha256') != object_digest(inputs)):
        raise BuildEvidenceError('build evidence does not match the current release or dependencies')
    prefix = Path(evidence.get('install_prefix', '/missing'))
    stage = entry/'.build/arm64'
    if (not install.is_symlink() or not prefix.is_relative_to(stage) or
            not prefix.resolve().is_relative_to(stage.resolve()) or
            install.resolve() != prefix.resolve() or not prefix.is_dir()):
        raise BuildEvidenceError('install symlink does not match this release build evidence')
    if binary_evidence(prefix, inputs['release']['controller_build']) != evidence.get('binaries'):
        raise BuildEvidenceError('binary or runtime library content differs from build evidence')
    if tree_fingerprint(prefix) != evidence.get('installed_files'):
        raise BuildEvidenceError('installed generated library or support files differ from build evidence')
    build = prefix.parent/'build'
    if cmake_evidence(build, inputs['dependencies']) != evidence.get('cmake'):
        raise BuildEvidenceError('CMake dependency evidence changed')
    return evidence


# 只读公开校验入口；必须先加载SDK环境，文件/格式错误统一转换明确构建拒绝。
def verify_build(entry: Path, sdk: Path | None = None) -> dict:
    """Read-only; caller must load navigation_environment.sh before invocation."""
    require_arm64()
    try:
        selected = sdk or os.environ.get('FC01_PROJECT_ROOT')
        if not selected:
            raise BuildEvidenceError('FC01_PROJECT_ROOT is missing; load navigation_environment.sh')
        return _verify(Path(entry), Path(selected))
    except (OSError, ValueError, KeyError, TypeError) as exc:
        raise BuildEvidenceError('build evidence verification failed: '+str(exc)) from exc


@contextmanager
# 锁覆盖整个离线构建/发布安装链接过程；非阻塞失败避免两次构建同时写同一证据。
def build_lock(entry):
    path = Path(entry)/'.build'
    path.mkdir(exist_ok=True)
    with (path/'arm64-build.lock').open('a') as stream:
        try:
            fcntl.flock(stream, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise BuildEvidenceError('another offline build is already running') from exc
        yield


# 独占临时文件写完flush/fsync后原子替换，finally只清自己的tmp，避免半写成功记录。
def atomic_json(path, value):
    temporary = Path(str(path)+'.tmp-'+str(os.getpid()))
    try:
        with temporary.open('x') as stream:
            json.dump(value, stream, ensure_ascii=False, indent=2); stream.write('\n')
            stream.flush(); os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


# 先尝试严格复用，失败保留原prefix并新建独立stage；编译前后再比输入，最终发布链接与证据。
def build_release(entry, sdk):
    require_arm64(); entry = Path(entry).resolve(); sdk = Path(sdk).resolve()
    with build_lock(entry):
        inputs = build_inputs(entry, sdk)
        try:
            evidence = _verify(entry, sdk, inputs)
        except (BuildEvidenceError, OSError, ValueError, KeyError, TypeError) as exc:
            print('FC01_ARM64_REBUILD_REQUIRED reason='+str(exc), flush=True)
        else:
            print('FC01_ARM64_BUILD_REUSED input_sha256='+evidence['input_sha256'], flush=True)
            return evidence
        install = entry/'install'
        if install.exists() and not install.is_symlink():
            raise BuildEvidenceError('existing install directory is retained; use the new independent release directory')
        if install.is_symlink() and not install.resolve().is_relative_to(entry/'.build/arm64'):
            raise BuildEvidenceError('unknown install symlink retained; no build target overwritten')
        base = entry/'.build/arm64'; base.mkdir(parents=True, exist_ok=True)
        stage = Path(tempfile.mkdtemp(prefix=object_digest(inputs)[:20]+'-', dir=base))
        prefix = stage/'install'; build = stage/'build'
        compiler = inputs['dependencies']['tools']
        commands = [
            [compiler['cmake']['path'], '-S', str(entry/'native/src/jikl_fc01_system'), '-B', str(build),
             '-G', 'Unix Makefiles', '-DCMAKE_BUILD_TYPE=Release', '-DCMAKE_INSTALL_PREFIX='+str(prefix),
             '-DCMAKE_CXX_COMPILER='+compiler['g++']['path'], '-DCMAKE_C_COMPILER='+compiler['gcc']['path'],
             '-DPython3_EXECUTABLE=/usr/bin/python3', '-DPYTHON_EXECUTABLE=/usr/bin/python3'],
            [compiler['cmake']['path'], '--build', str(build), '--parallel', '1'],
            [compiler['cmake']['path'], '--install', str(build)],
        ]
        for argv in commands:
            print('FC01_OFFLINE_BUILD_STEP '+str(argv[1]), flush=True)
            subprocess.run(argv, stdin=subprocess.DEVNULL, check=True)
        # 编译期间源码/依赖变化即拒绝，即使cmake成功也不发布伪对应的成功证据。
        if build_inputs(entry, sdk) != inputs:
            raise BuildEvidenceError('source or dependencies changed while compiling; partial build retained')
        evidence = dict(schema=SCHEMA, entry=str(entry), source_commit=inputs['release']['source_commit'],
                        host_architecture=platform.machine(), sdk_root=str(sdk),
                        controller_build=inputs['release']['controller_build'],
                        built_at_utc=datetime.now(timezone.utc).isoformat(),
                        input_sha256=object_digest(inputs), inputs=inputs, install_prefix=str(prefix),
                        build_dir=str(build), controller_executed=False, agent_or_service_started=False,
                        build_log=os.environ.get('FC01_BUILD_LOG', ''),
                        binaries=binary_evidence(prefix, inputs['release']['controller_build']),
                        installed_files=tree_fingerprint(prefix),
                        cmake=cmake_evidence(build, inputs['dependencies']))
        temporary = entry/('.install-new-'+str(os.getpid()))
        try:
            temporary.symlink_to(prefix.relative_to(entry), target_is_directory=True)
            os.replace(temporary, install)
        finally:
            temporary.unlink(missing_ok=True)
        atomic_json(entry/'ARM64_BUILD_EVIDENCE.json', evidence)
        print('FC01_ARM64_FRESH_BUILD_VERIFIED source_commit='+evidence['source_commit'], flush=True)
        return evidence


# 只分派build/verify；错误非零并输出证据路径，不调用生产控制节点。
def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=('build', 'verify'))
    parser.add_argument('--entry', type=Path, default=Path(__file__).resolve().parent)
    parser.add_argument('--sdk', type=Path)
    args = parser.parse_args(argv)
    try:
        sdk = args.sdk or os.environ.get('FC01_PROJECT_ROOT')
        if not sdk:
            raise BuildEvidenceError('FC01_PROJECT_ROOT is missing; load navigation_environment.sh')
        evidence = build_release(args.entry, sdk) if args.action == 'build' else verify_build(args.entry, sdk)
        if args.action == 'verify':
            print('FC01_ARM64_BUILD_EVIDENCE_VERIFIED input_sha256='+evidence['input_sha256'])
        print('FC01_ARM64_BUILD_EVIDENCE='+str(args.entry.resolve()/'ARM64_BUILD_EVIDENCE.json'))
        return 0
    except (BuildEvidenceError, OSError, ValueError, KeyError, TypeError, subprocess.SubprocessError) as exc:
        print('FC01_OFFLINE_BUILD_FAILED: '+str(exc), file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
