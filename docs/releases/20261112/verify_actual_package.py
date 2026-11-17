"""独立验收最终包；不导入打包器，只执行真实安装器的仅解包模式。"""
import argparse
import base64
import hashlib
import io
import json
import os
from pathlib import Path, PurePosixPath
import stat
import subprocess
import tempfile
import zipfile

parser = argparse.ArgumentParser()
parser.add_argument('package', type=Path)
parser.add_argument('repository', type=Path)
parser.add_argument('evidence', type=Path)
args = parser.parse_args()
package = args.package.resolve()
repository = args.repository.resolve()
evidence = args.evidence.resolve()
evidence.mkdir(parents=True, exist_ok=True)
blob = package.read_bytes()
sha = hashlib.sha256(blob).hexdigest()
assert Path(str(package) + '.sha256').read_text() == sha + '  ' + package.name + '\n'
marker = b'\n__FC01_PAYLOAD_BASE64__\n'
assert blob.count(marker) == 1
payload = base64.b64decode(b''.join(blob.split(marker, 1)[1].split()), validate=True)
with zipfile.ZipFile(io.BytesIO(payload)) as archive:
    assert archive.testzip() is None
    names = archive.namelist()
    assert len(names) == len(set(names)) <= 512
    files = {}
    for member in archive.infolist():
        path = PurePosixPath(member.filename)
        assert not path.is_absolute() and '..' not in path.parts
        assert not member.is_dir() and not stat.S_ISLNK(member.external_attr >> 16)
        assert member.file_size <= 1048576
        files[member.filename] = archive.read(member)
assert sum(map(len, files.values())) <= 8388608
manifest = json.loads(files['FC01_RELEASE.json'])
assert manifest['release_name'] == 'FC01_OrangePi_Navigation_20261112_release'
assert manifest['version'] == '20261112.1'
assert manifest['payload_profile'] == 'navigation-runtime-v1'
assert manifest['controller_build'] == 'FC01_RECOVERY_DIAGNOSTIC_V5_20261110'
assert manifest['working_tree_dirty'] is False
source_commit = manifest['source_commit']
assert len(source_commit) == 40
hashes = {n: hashlib.sha256(v).hexdigest() for n, v in files.items() if n != 'FC01_RELEASE.json'}
assert hashes == manifest['payload_sha256'] == manifest['source_sha256']
for name, data in files.items():
    if name == 'FC01_RELEASE.json':
        assert data == (repository / name).read_bytes()
        continue
    assert data == (repository / name).read_bytes(), name
    historical = subprocess.check_output(['git', '-C', str(repository), 'show', source_commit + ':' + name])
    assert data == historical, name
required = {
    'navigation_service.py', 'navigation_boot.sh', 'navigation.sh',
    'navigation_build_fresh.sh', 'navigation_environment.sh', 'navigation_build_evidence.py',
    'navigation_deploy.sh', 'navigation_deploy.py', 'navigation_install.py',
    'navigation_restore.sh', 'navigation_restore.py', 'navigation_status.py',
    'navigation_probe.py', 'navigation_collect.py', 'navigation_report.py',
    'launcher.py', 'standby.py', 'autostart.py', 'fc01_control.py',
    'navigation_settings.json', 'vision_settings.json', 'vision_settings.py', 'vision_config.py',
    'native/src/jikl_fc01_system/CMakeLists.txt', 'native/src/jikl_fc01_system/package.xml',
    'native/src/jikl_fc01_system/src/fc01_system_node.cpp',
    'native/src/jikl_fc01_system/src/fc01_config_check.cpp',
    'native/config/flight_test.yaml', 'native/config/diagnostic.yaml',
    'native/tools/fcctl.py', 'native/tools/socket_safety.py',
    'native/tools/vision_odometry_adapter.py', 'native/tools/velocity_command_diagnostic.py',
    'native/tools/vision_state_validation.py', 'native/tools/vision_resource_budget.py',
    'docs/FC01_ORANGEPI_RELEASE_20261112_zh.md',
}
for message in ('VisionState', 'VisionHealth', 'NavigationCommand', 'VelocityCommand', 'AttitudeThrustPreview'):
    required.add('native/src/jikl_fc01_system/msg/' + message + '.msg')
tracked_native = subprocess.check_output(['git', '-C', str(repository), 'ls-tree', '-r', '--name-only', source_commit, 'native/src', 'native/config']).decode().splitlines()
required.update(n for n in tracked_native if n.startswith('native/src/') or n.endswith('.yaml'))
assert required <= files.keys(), sorted(required - files.keys())
assert sum(n.endswith('.msg') for n in files) == 5
assert not any(n.startswith(('tests/', 'native/tests/', 'native/scripts/', 'docs/readability/', 'dist/')) for n in files)
assert not any('/__pycache__/' in n or n.endswith(('.pyc', '.zip')) for n in files)
assert not any(n.startswith('native/config/') and (n.endswith('.bak') or '.before_' in n) for n in files)
# 全量 native/src 保留 Git 中的两份纯文本历史备份；CMake 只编译当前两个显式 .cpp，绝不交付旧 ELF。
assert not any(v.startswith(b'\x7fELF') for v in files.values())
commands = []
home = Path(tempfile.mkdtemp(prefix='actual-release-home-', dir=evidence))
env = dict(os.environ, HOME=str(home), PYTHONDONTWRITEBYTECODE='1')
env.pop('FC01_NAVIGATION_INSTALL_DIR', None)
def run(script, mode, *, target=None, success=True, reason=None):
    current = dict(env)
    if target is not None:
        current['FC01_NAVIGATION_INSTALL_DIR'] = str(target)
    result = subprocess.run(['bash', str(script), mode], env=current, capture_output=True, text=True)
    commands.append({'argv': ['bash', str(script), mode], 'target': str(target) if target else None,
                     'returncode': result.returncode, 'stdout': result.stdout, 'stderr': result.stderr})
    assert (result.returncode == 0) is success, commands[-1]
    if reason:
        assert reason in result.stdout + result.stderr, commands[-1]
    return result
run(package, '--install-only', reason='FC01_EXTRACT_ONLY_COMPLETE')
entry = home / manifest['release_name']
for name, data in files.items():
    extracted = entry / name
    assert extracted.read_bytes() == data, name
    assert stat.S_IMODE(extracted.stat().st_mode) == (0o700 if name.endswith('.sh') else 0o600)
assert {p.relative_to(entry).as_posix() for p in entry.rglob('*') if p.is_file()} == files.keys()
run(package, '--extract-only', reason='FC01_PAYLOAD_VERIFIED_REUSED')
custom = home / 'custom installation'
run(package, '--install-only', target=custom)
changed = custom / 'navigation_settings.json'
changed.write_bytes(changed.read_bytes() + b'\nLOCAL_EDIT\n')
before = changed.read_bytes()
run(package, '--install-only', target=custom, success=False, reason='现有目录与本包不一致')
assert changed.read_bytes() == before
outside = home / 'outside'
outside.mkdir()
link = home / 'link'
link.symlink_to(outside, target_is_directory=True)
run(package, '--install-only', target=link / 'blocked', success=False, reason='符号链接')
assert not list(outside.iterdir())
tampered = evidence / 'tampered-release.sh'
tampered.write_bytes(blob + b'\n')
Path(str(tampered) + '.sha256').write_text(sha + '  ' + tampered.name + '\n')
run(tampered, '--install-only', target=home / 'tampered-target', success=False, reason='SHA256 不匹配')
assert not (home / 'tampered-target').exists()
result = {
    'package': str(package), 'package_bytes': len(blob), 'package_sha256': sha,
    'archive_sha256': hashlib.sha256(payload).hexdigest(), 'source_commit': source_commit,
    'working_tree_dirty_at_source_freeze': False, 'members': len(files),
    'uncompressed_bytes': sum(map(len, files.values())), 'required_files_checked': len(required),
    'all_payload_bytes_match_git_source_commit': True, 'independent_zip_crc_manifest_sha_ok': True,
    'actual_extracted_entry': str(entry), 'commands': commands, 'hardware_accessed': False,
}
(evidence / 'ACTUAL_PACKAGE_VERIFICATION.json').write_text(json.dumps(result, indent=2, ensure_ascii=False) + '\n')
print(json.dumps({k:v for k,v in result.items() if k != 'commands'}, indent=2, ensure_ascii=False))
