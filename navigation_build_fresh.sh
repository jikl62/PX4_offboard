#!/usr/bin/env bash
# Native ARM64 build in a freshly extracted release; never starts a controller.
set -euo pipefail
case "$(uname -m)" in
  aarch64|arm64) ;;
  *) echo 'FC01_FRESH_BUILD_REFUSED: native ARM64 host required' >&2; exit 2 ;;
esac
fc01_fresh_entry=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
fc01_fresh_sdk=${FC01_PROJECT_ROOT:?Set FC01_PROJECT_ROOT to the existing ROS/px4_msgs SDK root}
export FC01_PROJECT_ROOT="$fc01_fresh_sdk"
/usr/bin/python3 - "$fc01_fresh_entry" <<'PY'
import hashlib, json, pathlib, re, sys
root = pathlib.Path(sys.argv[1]).resolve()
for name in ('install', '.build', 'build-complete.json', 'ARM64_BUILD_EVIDENCE.json'):
    path = root/name
    if path.exists() or path.is_symlink():
        raise SystemExit('FC01_FRESH_BUILD_REFUSED: existing build/evidence path '+str(path))
release = json.loads((root/'FC01_RELEASE.json').read_text())
if not re.fullmatch(r'[0-9a-f]{40}', str(release.get('source_commit', ''))):
    raise SystemExit('FC01_FRESH_BUILD_REFUSED: missing/invalid source_commit')
files = release.get('source_sha256')
required = ('navigation_build_fresh.sh', 'navigation_boot.sh', 'navigation_install.py',
    'native/src/jikl_fc01_system/CMakeLists.txt',
    'native/src/jikl_fc01_system/src/fc01_system_node.cpp',
    'native/src/jikl_fc01_system/src/fc01_config_check.cpp',
    'native/src/jikl_fc01_system/include/jikl_fc01_system/discrete_event_latch.hpp',
    'native/src/jikl_fc01_system/include/jikl_fc01_system/runtime.hpp',
    'native/src/jikl_fc01_system/include/jikl_fc01_system/attitude_altitude_core.hpp')
if not isinstance(files, dict) or any(name not in files for name in required):
    raise SystemExit('FC01_FRESH_BUILD_REFUSED: incomplete source_sha256 manifest')
for name, expected in files.items():
    relative = pathlib.PurePosixPath(name)
    path = root/name
    if (relative.is_absolute() or '..' in relative.parts or path.is_symlink() or
            not path.resolve().is_relative_to(root) or not path.is_file() or
            not re.fullmatch(r'[0-9a-f]{64}', str(expected)) or
            hashlib.sha256(path.read_bytes()).hexdigest() != expected):
        raise SystemExit('FC01_FRESH_BUILD_REFUSED: source hash/path mismatch '+name)
print('FC01_RELEASE_VERIFIED source_commit='+release['source_commit'], flush=True)
PY
# ROS/colcon setup scripts may read unset optional variables.
set +u
source /opt/ros/humble/setup.bash
source "$fc01_fresh_sdk/install_r632/local_setup.bash"
set -u
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 PYTHONUNBUFFERED=1
bash "$fc01_fresh_entry/navigation_boot.sh" build \
  --prefix "$fc01_fresh_entry/install" --build-dir "$fc01_fresh_entry/.build/arm64-final"
/usr/bin/python3 - "$fc01_fresh_entry" "$fc01_fresh_sdk" <<'PY'
import datetime, hashlib, importlib, json, os, pathlib, platform, re, shutil, subprocess, sys
import xml.etree.ElementTree as ET
root = pathlib.Path(sys.argv[1]).resolve()
sdk = pathlib.Path(sys.argv[2]).expanduser().resolve()
release_path = root/'FC01_RELEASE.json'
release = json.loads(release_path.read_text())
# Reverify committed files after compilation, before asserting provenance.
for name, expected in release['source_sha256'].items():
    if hashlib.sha256((root/name).read_bytes()).hexdigest() != expected:
        raise SystemExit('FC01_FRESH_BUILD_FAILED: source changed during build '+name)
def command(args):
    result = subprocess.run(args, capture_output=True, text=True, check=True)
    return result.stdout.strip()
evidence = {'source_commit': release['source_commit'], 'host_architecture': platform.machine(),
    'built_at_utc': datetime.datetime.now(datetime.timezone.utc).isoformat(),
    'release_manifest_sha256': hashlib.sha256(release_path.read_bytes()).hexdigest(),
    'entry': str(root), 'sdk_root': str(sdk), 'build_dir': str(root/'.build/arm64-final'),
    'controller_executed': False, 'agent_or_service_started': False, 'binaries': {}}
marker = b'R2_DISCRETE_EVENT_PREFLIGHT_V2_20261108'
for name in ('fc01_system_node', 'fc01_config_check'):
    binary = root/'install/lib/jikl_fc01_system'/name
    data = binary.read_bytes()
    if (len(data) < 64 or data[:6] != b'\x7fELF\x02\x01' or
            int.from_bytes(data[18:20], 'little') != 183 or not os.access(binary, os.X_OK)):
        raise SystemExit('FC01_FRESH_BUILD_FAILED: not executable AArch64 ELF '+str(binary))
    if name == 'fc01_system_node' and marker not in data:
        raise SystemExit('FC01_FRESH_BUILD_FAILED: missing compiled safety marker')
    resolved = subprocess.run(['ldd', str(binary)], capture_output=True, text=True)
    report = resolved.stdout+resolved.stderr
    if resolved.returncode or 'not found' in report:
        raise SystemExit('FC01_FRESH_BUILD_FAILED: unresolved runtime libraries\n'+report)
    libraries = []
    for line in report.splitlines():
        match = re.search(r'(?:=>\s+)?(/\S+)\s+\(', line)
        if match:
            library = pathlib.Path(match.group(1)).resolve()
            libraries.append({'path': str(library),
                              'sha256': hashlib.sha256(library.read_bytes()).hexdigest()})
    evidence['binaries'][name] = {'path': str(binary), 'elf_machine': 'AArch64',
        'sha256': hashlib.sha256(data).hexdigest(), 'ldd': report,
        'resolved_libraries': libraries,
        'safety_marker': marker.decode() if name == 'fc01_system_node' else None}
px4 = importlib.import_module('px4_msgs')
rclpy = importlib.import_module('rclpy')
px4_prefix = pathlib.Path(command(['ros2', 'pkg', 'prefix', 'px4_msgs'])).resolve()
if not px4_prefix.is_relative_to((sdk/'install_r632').resolve()):
    raise SystemExit('FC01_FRESH_BUILD_FAILED: px4_msgs resolved outside selected SDK '+str(px4_prefix))
cache = root/'.build/arm64-final/CMakeCache.txt'
dependency_dirs = {}
for line in cache.read_text().splitlines():
    if re.match(r'(px4_msgs|rclcpp|ament_cmake|builtin_interfaces)_DIR:PATH=', line):
        key, value = line.split('=', 1)
        dependency_dirs[key.split(':', 1)[0]] = value
if pathlib.Path(dependency_dirs.get('px4_msgs_DIR', '/missing')).resolve() != px4_prefix/'share/px4_msgs/cmake':
    raise SystemExit('FC01_FRESH_BUILD_FAILED: CMake/Python ROS SDK dependency mismatch')
package = px4_prefix/'share/px4_msgs/package.xml'
evidence['dependencies'] = {'ros_distro': os.environ.get('ROS_DISTRO'),
    'ros2_executable': shutil.which('ros2'),
    'rclcpp_version': ET.fromstring(command(['ros2', 'pkg', 'xml', 'rclcpp'])).findtext('version'),
    'px4_msgs_prefix': str(px4_prefix), 'px4_msgs_python': list(map(str, px4.__path__)),
    'px4_msgs_package_sha256': hashlib.sha256(package.read_bytes()).hexdigest(),
    'px4_msgs_version': ET.fromstring(package.read_text()).findtext('version'),
    'rclpy_python': str(pathlib.Path(rclpy.__file__).resolve()),
    'cmake_dependency_dirs': dependency_dirs, 'cmake_cache_sha256': hashlib.sha256(cache.read_bytes()).hexdigest(),
    'compiler': command(['g++', '--version']), 'cmake': command(['cmake', '--version']),
    'environment': {key: os.environ.get(key, '') for key in
        ('ROS_DISTRO', 'AMENT_PREFIX_PATH', 'CMAKE_PREFIX_PATH', 'LD_LIBRARY_PATH', 'PYTHONPATH')}}
if evidence['dependencies']['ros_distro'] != 'humble':
    raise SystemExit('FC01_FRESH_BUILD_FAILED: ROS_DISTRO must be humble')
target = root/'ARM64_BUILD_EVIDENCE.json'
with target.open('x') as output:
    json.dump(evidence, output, indent=2)
    output.write('\n')
print('FC01_ARM64_FRESH_BUILD_VERIFIED source_commit='+release['source_commit'])
for name, binary in evidence['binaries'].items():
    print(name+' sha256='+binary['sha256'])
print('FC01_ARM64_BUILD_EVIDENCE='+str(target))
PY
