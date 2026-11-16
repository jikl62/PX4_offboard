#!/usr/bin/env python3
"""Run the packaged ARM64 controller with an inherited, unchanged RC policy."""
from __future__ import annotations

import argparse
import copy
import hashlib
import importlib
import json
import math
import os
from pathlib import Path, PurePosixPath
import platform
import re
import signal
import subprocess
import sys
import tempfile
import threading

from fc01_control import ControlError
from navigation_service import check_controller, describe_row
import standby
import yaml


ENTRY = Path(__file__).resolve().parent
CONTROLLER = 'install/lib/jikl_fc01_system/fc01_system_node'
OVERLAY_KEYS = frozenset(
    ('AMENT_PREFIX_PATH', 'COLCON_PREFIX_PATH', 'CMAKE_PREFIX_PATH', 'PYTHONPATH',
     'LD_LIBRARY_PATH', 'ROS_PACKAGE_PATH', 'ROS_DISTRO', 'COLCON_CURRENT_PREFIX',
     'AMENT_CURRENT_PREFIX')
)
RESERVED_KEYS = frozenset(
    ('HOME', 'USER', 'LOGNAME', 'SHELL', 'PATH', 'PWD', 'OLDPWD', 'SHLVL', '_',
     'BASH_ENV', 'ENV', 'SHELLOPTS', 'BASHOPTS', 'LD_PRELOAD', 'LD_AUDIT')
)


def _sha256(path):
    value = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            value.update(block)
    return value.hexdigest()


def _relative(name):
    path = PurePosixPath(name)
    if (not name or path.is_absolute() or '..' in path.parts or '\\' in name
            or path.as_posix() != name):
        raise ControlError('invalid packaged path: ' + name)
    return path


def release_manifest(entry=ENTRY):
    """Verify immutable payload bytes before using its executable or modules."""
    entry = Path(entry).resolve()
    manifest = json.loads((entry / 'ARM64_RELEASE.json').read_text())
    if (not isinstance(manifest, dict)
            or not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9._-]*', str(manifest.get('version', '')))
            or not re.fullmatch(r'[0-9a-f]{40}', str(manifest.get('source_commit', '')))
            or manifest.get('architecture') != 'aarch64'
            or manifest.get('ubuntu') != '22.04'
            or manifest.get('ros_distro') != 'humble'
            or manifest.get('controller_relative') != CONTROLLER
            or not isinstance(manifest.get('payload_sha256'), dict)):
        raise ControlError('unsupported ARM64 release manifest')
    files = manifest['payload_sha256']
    required = (CONTROLLER, str(PurePosixPath(CONTROLLER).parent / 'fc01_config_check'),
                'prebuilt_boot.sh', 'navigation_prebuilt_service.py')
    if any(name not in files for name in required):
        raise ControlError('incomplete ARM64 payload manifest')
    for name, expected in files.items():
        _relative(name)
        path = entry / name
        if (name == 'ARM64_RELEASE.json' or PurePosixPath(name).parts[0] == 'deployment'
                or not re.fullmatch(r'[0-9a-f]{64}', str(expected))
                or not path.is_file() or not path.resolve().is_relative_to(entry)
                or _sha256(path) != expected):
            raise ControlError('packaged file hash/path mismatch: ' + name)
    for path in entry.rglob('*'):
        relative = path.relative_to(entry)
        if relative.parts[0] == 'deployment' or '__pycache__' in relative.parts:
            continue
        if path.is_symlink() and not path.resolve().is_relative_to(entry):
            raise ControlError('packaged symlink escapes release: ' + str(relative))
        if path.is_file() and str(relative) != 'ARM64_RELEASE.json' and str(relative) not in files:
            raise ControlError('unlisted packaged file: ' + str(relative))
    return manifest


def deployment_environment(entry=ENTRY):
    """Load inherited transport settings; setup-generated paths belong to this bundle."""
    entry = Path(entry).resolve()
    path = entry / 'deployment/environment.json'
    value = json.loads(path.read_text())
    if not isinstance(value, dict) or not isinstance(value.get('environment', {}), dict):
        raise ControlError('invalid deployment environment')
    inherited = value.get('environment', {})
    for key, item in inherited.items():
        if (not isinstance(key, str) or not re.fullmatch(r'[A-Za-z_][A-Za-z0-9_]*', key)
                or not isinstance(item, str) or '\0' in item):
            raise ControlError('invalid inherited environment entry')
    for key in ('ROS_DOMAIN_ID', 'ROS_LOCALHOST_ONLY'):
        if inherited.get(key, '0') != '0':
            raise ControlError('prebuilt service requires ' + key + '=0')
    if inherited.get('ROS_DISTRO', 'humble') != 'humble':
        raise ControlError('prebuilt service requires ROS_DISTRO=humble')
    if (inherited.get('FC01_DEVICE', '/dev/ttyS2') != '/dev/ttyS2'
            or inherited.get('FC01_BAUD', '115200') != '115200'):
        raise ControlError('prebuilt service requires existing /dev/ttyS2 Agent at 115200')
    if value.get('mode', 'flight-test') != 'flight-test':
        raise ControlError('prebuilt service requires inherited manual RC flight-test mode')
    agent = Path(value.get('agent_path') or Path.home() / 'fc01_agent_build_offline/MicroXRCEAgent')
    if not agent.is_absolute():
        raise ControlError('inherited Agent path must be absolute')
    value['agent_path'] = str(agent)
    value['environment'] = {
        key: item for key, item in inherited.items()
        if key not in OVERLAY_KEYS and key not in RESERVED_KEYS and not key.startswith('BASH_FUNC_')
    }
    value['environment'].setdefault('ROS_DOMAIN_ID', '0')
    value['environment'].setdefault('ROS_LOCALHOST_ONLY', '0')
    return value


def validate_environment(environ=None):
    environ = os.environ if environ is None else environ
    if platform.machine().lower() not in ('aarch64', 'arm64'):
        raise ControlError('prebuilt runtime requires native ARM64')
    if environ.get('ROS_DISTRO') != 'humble':
        raise ControlError('prebuilt runtime requires ROS_DISTRO=humble')
    if any(environ.get(key, '0') != '0' for key in ('ROS_DOMAIN_ID', 'ROS_LOCALHOST_ONLY')):
        raise ControlError('prebuilt runtime requires ROS_DOMAIN_ID=0 and ROS_LOCALHOST_ONLY=0')


def snapshot_profile(entry, manifest, *, config_path=None, home=None):
    """Copy every inherited key, updating only the three runtime identity fields."""
    entry = Path(entry).resolve()
    source = Path(config_path) if config_path is not None else entry / 'deployment/runtime.yaml'
    base = yaml.safe_load(source.read_text())
    if (not isinstance(base, dict) or not isinstance(base.get('fc01_system'), dict)
            or not isinstance(base['fc01_system'].get('ros__parameters'), dict)):
        raise ControlError('inherited runtime lacks fc01_system/ros__parameters')
    profile = copy.deepcopy(base)
    params = profile['fc01_system']['ros__parameters']
    if params.get('profile') != 'attitude_altitude' or params.get('use_sim_time', False) is not False:
        raise ControlError('inherited runtime must use real attitude_altitude profile')
    for name in ('flight_enabled', 'arm_enabled', 'visual_navigation_enabled',
                 'bench_arm_only_enabled', 'autonomous_failsafe_confirmed'):
        if params.get(name) is not False:
            raise ControlError('inherited manual RC runtime requires ' + name + '=false')
    for name in ('rc_navigation_enabled', 'flight_actions_enabled', 'takeoff_enabled',
                 'flight_parameters_confirmed', 'rc_takeover_confirmed'):
        if params.get(name) is not True:
            raise ControlError('inherited manual RC runtime requires existing ' + name + '=true')
    slot = params.get('rc_navigation_mode_slot')
    target, maximum = params.get('rc_target_height'), params.get('aa_max_height')
    if type(slot) is not int or not 1 <= slot <= 6:
        raise ControlError('invalid inherited RC navigation slot')
    if (type(target) not in (int, float) or type(maximum) not in (int, float)
            or not math.isfinite(target) or not math.isfinite(maximum) or not 0 < target < maximum):
        raise ControlError('invalid inherited RC target height')
    home = Path.home() if home is None else Path(home)
    params.update(
        fc01_entry_version=manifest['version'],
        fc01_config_path=str((home / 'FC01_navigation/runtime.yaml').resolve()),
        fc01_config_source_path=str((entry / 'deployment/runtime.yaml').resolve()),
    )
    return profile


def _verify_elf(path):
    with Path(path).open('rb') as stream:
        header = stream.read(64)
    if (len(header) < 64 or header[:6] != b'\x7fELF\x02\x01'
            or int.from_bytes(header[18:20], 'little') != 183):
        raise ControlError('packaged file is not ARM64 ELF: ' + str(path))


def _command(argv, run):
    result = run(list(map(str, argv)), stdin=subprocess.DEVNULL, capture_output=True,
                 text=True, timeout=45, check=False)
    if result.returncode or 'not found' in (result.stdout + result.stderr):
        raise ControlError('prebuilt validation failed: ' + str(argv) + '\n'
                           + result.stdout + result.stderr)
    return result.stdout


def check_type_support(entry):
    """Require both Python message modules and their native support from this install."""
    prefix = (Path(entry) / 'install').resolve()
    from rclpy.type_support import check_for_type_support
    packages = {
        'px4_msgs.msg': ('BatteryStatus', 'VehicleAttitude', 'VehicleLocalPosition',
                         'VehicleStatus', 'VehicleLandDetected', 'FailsafeFlags',
                         'ManualControlSetpoint', 'ManualControlSwitches', 'VehicleCommandAck',
                         'VehicleCommand', 'VehicleAttitudeSetpoint', 'OffboardControlMode',
                         'TrajectorySetpoint'),
        'jikl_fc01_system.msg': ('VisionState', 'NavigationCommand', 'VelocityCommand',
                                 'VisionHealth', 'AttitudeThrustPreview'),
    }
    for package, names in packages.items():
        module = importlib.import_module(package)
        if not Path(module.__file__).resolve().is_relative_to(prefix):
            raise ControlError('message module is outside bundled install: ' + package)
        for name in names:
            check_for_type_support(getattr(module, name))


def check(entry=ENTRY, *, config_path=None, run=subprocess.run):
    """Read-only payload/dependency/config checks; never run the controller or Agent."""
    entry = Path(entry).resolve()
    manifest = release_manifest(entry)
    deployment_environment(entry)
    validate_environment()
    binary = check_controller(entry=entry)
    checker = binary.parent / 'fc01_config_check'
    if not os.access(checker, os.X_OK):
        raise ControlError('packaged native configuration checker is not executable')
    # All installed native libraries must match the package architecture, including
    # generated Python extensions; ldd must resolve each shared dependency locally.
    native_files = []
    for name in manifest['payload_sha256']:
        path = entry / name
        with path.open('rb') as stream:
            is_elf = stream.read(4) == b'\x7fELF'
        if is_elf:
            _verify_elf(path)
            native_files.append(path)
    for path in native_files:
        _command(['/usr/bin/ldd', path], run)
    check_type_support(entry)
    profile = snapshot_profile(entry, manifest, config_path=config_path)
    with tempfile.TemporaryDirectory(prefix='fc01_prebuilt_check_') as temporary:
        config = Path(temporary) / 'runtime.yaml'
        config.write_text(yaml.safe_dump(profile, allow_unicode=True, sort_keys=False))
        # The checker creates only its parameter-validation node; it does not spin
        # or create flight publishers and never touches the serial transport.
        _command([checker, '--ros-args', '--disable-external-lib-logs',
                  '--params-file', config], run)
    return manifest, profile


def run_service(entry=ENTRY):
    entry = Path(entry).resolve()
    manifest, profile = check(entry)
    inherited = deployment_environment(entry)
    agent = Path(inherited['agent_path'])
    source = entry / 'deployment/runtime.yaml'
    snapshot_sha256 = _sha256(source)
    params = profile['fc01_system']['ros__parameters']
    stopped = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, lambda *_: stopped.set())

    def exact_runtime_profile(_root, _agent):
        # Each restart revalidates immutable bytes and reloads the snapshot.
        current = release_manifest(entry)
        if current != manifest:
            raise ControlError('prebuilt release identity changed during supervision')
        if _sha256(source) != snapshot_sha256:
            raise ControlError('inherited runtime snapshot changed during supervision')
        check_controller(entry=entry)
        return snapshot_profile(entry, current)

    metadata = {
        key: params.get(key) for key in
        ('rc_navigation_enabled', 'flight_enabled', 'flight_actions_enabled', 'arm_enabled',
         'takeoff_enabled', 'rc_target_height', 'aa_hover_thrust', 'aa_takeoff_reserve_thrust')
    }
    metadata.update(
        runtime_mode='flight-test', entry_version=manifest['version'],
        source_commit=manifest['source_commit'], candidate_commit=manifest['source_commit'],
        config_path=params['fc01_config_path'], config_source_path=str(source),
        automatic_arm=False, visual_navigation_enabled=False,
    )
    print('FC01_PREBUILT_SERVICE version=' + manifest['version']
          + ' source_commit=' + manifest['source_commit'] + ' inherited_config=' + str(source),
          flush=True)
    standby.supervise(
        entry, agent, Path.home() / 'FC01_navigation', stopped,
        controller=entry / manifest['controller_relative'], config_factory=exact_runtime_profile,
        status_metadata=metadata, config_path=source, row_formatter=describe_row,
    )


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=('check', 'run', 'environment'), nargs='?', default='run')
    parser.add_argument('--entry', type=Path, default=ENTRY)
    parser.add_argument('--config', type=Path)
    args = parser.parse_args(argv)
    if args.config is not None and args.action != 'check':
        parser.error('--config is only accepted by check')
    try:
        if args.action == 'environment':
            value = deployment_environment(args.entry)
            for key, item in value['environment'].items():
                sys.stdout.buffer.write((key + '=' + item).encode() + b'\0')
        elif args.action == 'check':
            manifest, _ = check(args.entry, config_path=args.config)
            print('FC01_PREBUILT_CHECK_OK version=' + manifest['version'])
        else:
            os.umask(0o077)
            run_service(args.entry)
        return 0
    except (OSError, ValueError, TypeError, KeyError, ControlError,
            ImportError, AttributeError, subprocess.SubprocessError) as exc:
        print('FC01_PREBUILT_SERVICE_FAILED: ' + str(exc), file=sys.stderr, flush=True)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
