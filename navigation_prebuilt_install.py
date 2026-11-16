#!/usr/bin/env python3
"""FC01 预编译 ARM64 发行安装事务；不编译、不下载、不解锁，不强杀控制器。

包内源码/运行文件不可变；deployment 是本机继承配置和事务记录。唯一允许的服务变更
是受管 unit 的专用 drop-in；失败时先等待新控制器安全退出，再还原原配置/运行状态。
Backend 可注入纯测试替身；命令行没有绕过架构或地面检查的测试开关。
"""
from __future__ import annotations

import argparse
import ctypes
from datetime import datetime, timezone
import errno
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path, PurePosixPath
import platform
import pwd
import re
import shlex
import shutil
import stat
import subprocess
import sys
import tarfile
import tempfile
import time

SERVICE = 'fc01-navigation.service'
MARKER = '# FC01_RC_NAVIGATION_MANUAL_ARM_20261028_V1'
DROPIN = 'zzzz-fc01-prebuilt.conf'
MANIFEST = 'ARM64_RELEASE.json'
MAX_ARCHIVE = 1024 * 1024 * 1024
MAX_TOTAL = 4 * 1024 * 1024 * 1024
MAX_FILES = 30000
# 安装验收需等完整payload/type support/ldd预检；与飞控反馈TTL和控制循环时限无关。
# ARM64 TCG测得单次启动预检46.377s，30s会误判未启动；真实原生启动仍按已有保护执行。
STARTUP_ACCEPTANCE_SECONDS = 180
ENV_KEYS = ('ROS_DOMAIN_ID', 'ROS_LOCALHOST_ONLY', 'RMW_IMPLEMENTATION',
            'ROS_DISCOVERY_SERVER', 'ROS_STATIC_PEERS', 'CYCLONEDDS_URI',
            'FASTRTPS_DEFAULT_PROFILES_FILE', 'FASTDDS_DEFAULT_PROFILES_FILE',
            'RMW_FASTRTPS_USE_QOS_FROM_XML', 'ROS_AUTOMATIC_DISCOVERY_RANGE',
            'ROS_SECURITY_ENABLE', 'ROS_SECURITY_STRATEGY', 'ROS_SECURITY_KEYSTORE')


class InstallError(RuntimeError):
    """显式失败；不会被转换成“已成功切换”。"""


def digest(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def atomic_json(path, value):
    path = Path(path)
    path.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
    fd, name = tempfile.mkstemp(prefix='.' + path.name, dir=path.parent)
    try:
        with os.fdopen(fd, 'w') as stream:
            json.dump(value, stream, ensure_ascii=False, indent=2)
            stream.write('\n')
            stream.flush()
            os.fsync(stream.fileno())
        os.chmod(name, 0o600)
        os.replace(name, path)
    finally:
        if os.path.exists(name):
            os.unlink(name)


def safe_name(name):
    p = PurePosixPath(name)
    return (bool(name) and not p.is_absolute() and '..' not in p.parts
            and '\\' not in name and name == p.as_posix())


def safe_directory(path, uid):
    """只接受本用户独占的目录祖先，禁止链接把发行写到另一位置。"""
    path = Path(path).absolute()
    for item in (path, *path.parents):
        if item.is_symlink():
            raise InstallError('安装路径包含符号链接：' + str(item))
        if item.exists() and not item.is_dir():
            raise InstallError('安装路径被文件占用：' + str(item))
    if path.exists() and (path.stat().st_uid != uid or path.stat().st_mode & 0o022):
        raise InstallError('安装目录所有者/权限不安全：' + str(path))


def verify_elf(path):
    with Path(path).open('rb') as stream:
        data = stream.read(64)
    if data[:4] != b'\x7fELF':
        return False
    if (len(data) < 64 or data[:6] != b'\x7fELF\x02\x01'
            or int.from_bytes(data[18:20], 'little') != 183):
        raise InstallError('包内存在非 AArch64 ELF：' + str(path))
    return True


def verify_tree(entry, manifest):
    if not isinstance(manifest,dict):
        raise InstallError('ARM64 发行清单不是对象')
    files = manifest.get('payload_sha256')
    if (not isinstance(files, dict) or not files
            or not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.-]{0,95}', str(manifest.get('version', '')))
            or not re.fullmatch(r'[0-9a-f]{40}', str(manifest.get('source_commit', '')))
            or manifest.get('architecture') != 'aarch64' or manifest.get('ubuntu') != '22.04'
            or manifest.get('ros_distro') != 'humble'):
        raise InstallError('ARM64 发行清单身份不完整')
    actual = set()
    for p in entry.rglob('*'):
        relative = p.relative_to(entry).as_posix()
        if p.is_symlink() or (not p.is_file() and not p.is_dir()):
            raise InstallError('发行树包含特殊/链接文件：' + relative)
        if p.is_file() and relative != MANIFEST and not relative.startswith('deployment/'):
            actual.add(relative)
    if set(files) != actual:
        raise InstallError('发行文件集合与清单不一致')
    for name, expected in files.items():
        if (not safe_name(name) or name == MANIFEST or name.startswith('deployment/')
                or not re.fullmatch(r'[0-9a-f]{64}', str(expected))
                or digest(entry / name) != expected):
            raise InstallError('发行内容摘要不一致：' + str(name))
        verify_elf(entry / name)
    relative = manifest.get('controller_relative')
    if not isinstance(relative, str) or relative not in files or not verify_elf(entry / relative):
        raise InstallError('缺少清单指定的 AArch64 控制器')
    if not os.access(entry / relative, os.X_OK):
        raise InstallError('控制器缺少执行权限')
    for name in ('prebuilt_boot.sh', 'navigation_prebuilt_service.py'):
        if name not in files:
            raise InstallError('缺少预编译服务入口：' + name)
    return entry / relative


def extract_checked(archive_path, expected, workspace):
    """严格核对压缩包和完整展开文件集；链接一律拒绝，发行器应解引用合法安装链接。"""
    archive_path = Path(archive_path)
    if (archive_path.is_symlink() or not archive_path.is_file()
            or archive_path.stat().st_size > MAX_ARCHIVE
            or not re.fullmatch(r'[0-9a-f]{64}', expected or '')
            or digest(archive_path) != expected):
        raise InstallError('安装包 SHA-256 不匹配或文件无效')
    names = set()
    top = None
    total = 0
    with tarfile.open(archive_path, 'r:gz') as archive:
        for member in archive:
            name = member.name.rstrip('/')
            if (not safe_name(name) or name in names or member.issym() or member.islnk()
                    or not (member.isfile() or member.isdir())):
                raise InstallError('不安全的归档成员：' + name)
            names.add(name)
            if len(names) > MAX_FILES:
                raise InstallError('归档成员数超限')
            parts = PurePosixPath(name).parts
            if top is None:
                top = parts[0]
            if (parts[0] != top or not re.fullmatch(r'FC01_OrangePi_[A-Za-z0-9_.-]+_arm64', top)
                    or (member.isfile() and len(parts) < 2) or 'deployment' in parts[1:2]):
                raise InstallError('归档顶层或本机配置目录无效')
            dest = workspace.joinpath(*parts)
            if member.isdir():
                dest.mkdir(mode=0o700, parents=True, exist_ok=True)
                continue
            total += member.size
            if member.size < 0 or member.size > MAX_ARCHIVE or total > MAX_TOTAL:
                raise InstallError('展开体积超限')
            dest.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
            source = archive.extractfile(member)
            if source is None:
                raise InstallError('无法读取归档文件')
            with source, dest.open('xb') as output:
                copied = 0
                while True:
                    chunk = source.read(1024 * 1024)
                    if not chunk:
                        break
                    copied += len(chunk)
                    if copied > member.size:
                        raise InstallError('归档成员长度超限')
                    output.write(chunk)
            if copied != member.size:
                raise InstallError('归档成员长度不符')
            dest.chmod(0o700 if member.mode & 0o111 else 0o600)
    if top is None:
        raise InstallError('安装包为空')
    entry = workspace / top
    for path in [entry, *(p for p in entry.rglob('*') if p.is_dir())]:
        path.chmod(0o700)
    try:
        manifest = json.loads((entry / MANIFEST).read_text())
    except (OSError, ValueError) as exc:
        raise InstallError('缺少有效发行清单') from exc
    verify_tree(entry, manifest)
    return entry, manifest


def publish_directory(source, target):
    """Linux renameat2 NOREPLACE：即使外部并发创建目录也不覆盖。"""
    libc = ctypes.CDLL(None, use_errno=True)
    rename = getattr(libc, 'renameat2', None)
    if rename is None:
        raise InstallError('系统缺少 renameat2，未覆盖任何目录')
    rename.argtypes = [ctypes.c_int, ctypes.c_char_p, ctypes.c_int, ctypes.c_char_p, ctypes.c_uint]
    if rename(-100, os.fsencode(source), -100, os.fsencode(target), 1):
        number = ctypes.get_errno()
        if number == errno.EEXIST:
            raise InstallError('版本目录被并发创建，未覆盖')
        raise OSError(number, os.strerror(number), str(target))


def systemd_quote(value):
    if any(c in str(value) for c in '\n\r\x00'):
        raise InstallError('systemd 路径含控制字符')
    return '"' + str(value).replace('\\', '\\\\').replace('"', '\\"').replace('%', '%%') + '"'


def systemd_directory(value):
    """WorkingDirectory是单个路径值，不使用ExecStart的shell词法双引号。

    systemd249会把这里的双引号保留为路径字符；百分号仍需转义以免展开specifier。
    拒绝控制字/反斜杠和边缘空白，避免unit续行/转义改变实际目录。
    """
    text=str(value)
    if (not Path(text).is_absolute() or text!=text.strip() or '\\' in text
            or any(ord(c)<32 or ord(c)==127 for c in text)):
        raise InstallError('systemd工作目录必须是无控制字的明确绝对路径')
    return text.replace('%','%%')


class Backend:
    """唯一系统副作用边界；测试替身只替换此对象，不向生产CLI开放跳过开关。"""

    def __init__(self, home=None):
        self.uid = os.getuid()
        if self.uid == 0:
            raise InstallError('请以 orangepi 普通用户运行，不要 sudo 整个安装器；写服务时会请求 sudo')
        account = pwd.getpwuid(self.uid)
        self.user = account.pw_name
        self.home = Path(home or account.pw_dir).resolve()
        self.units = Path('/etc/systemd/system')
        self.proc = Path('/proc')

    def run(self, argv, **kw):
        return subprocess.run(argv, check=True, text=True, capture_output=True, **kw)

    def privileged(self, argv):
        return self.run(['sudo', '-n', *map(str, argv)])

    def authenticate(self):
        subprocess.run(['sudo', '-v'], check=True)

    def check_host(self):
        if platform.machine().lower() not in ('aarch64', 'arm64'):
            raise InstallError('仅支持实际 ARM64 主机，未变更服务')
        release = dict(line.split('=', 1) for line in Path('/etc/os-release').read_text().splitlines() if '=' in line)
        if release.get('ID', '').strip('"') != 'ubuntu' or release.get('VERSION_ID', '').strip('"') != '22.04':
            raise InstallError('仅支持 Ubuntu 22.04，未变更服务')
        for name in ('/opt/ros/humble/setup.bash', '/usr/bin/systemctl', '/usr/bin/python3'):
            if not Path(name).is_file():
                raise InstallError('缺少目标依赖：' + name)
        import yaml  # noqa: F401

    def service(self):
        props = 'FragmentPath,DropInPaths,MainPID,ActiveState,SubState,UnitFileState,WorkingDirectory,User,Environment,EnvironmentFiles,ExecStart'
        text = self.run(['/usr/bin/systemctl', 'show', SERVICE, '--property=' + props]).stdout
        return dict(line.split('=', 1) for line in text.splitlines() if '=' in line)

    def process(self, pid):
        root = self.proc / str(pid)
        return dict(pid=pid, uid=root.stat().st_uid,
                    argv=[x.decode() for x in (root / 'cmdline').read_bytes().split(b'\0') if x],
                    exe=str((root / 'exe').resolve(strict=True)), cwd=str((root / 'cwd').resolve(strict=True)),
                    sha256=digest(root / 'exe'))

    def controllers(self):
        found = []
        for item in self.proc.iterdir():
            if not item.name.isdecimal():
                continue
            try:
                cmd = (item / 'cmdline').read_bytes().split(b'\0')
                if any(Path(x.decode(errors='replace')).name == 'fc01_system_node' for x in cmd[:2]):
                    found.append(self.process(int(item.name)))
            except (FileNotFoundError, ProcessLookupError):
                continue
        return found

    def descendant(self, pid, main):
        seen = set()
        while pid > 1 and pid not in seen:
            if pid == main:
                return True
            seen.add(pid)
            try:
                pid = int(next(line.split()[1] for line in (self.proc / str(pid) / 'status').read_text().splitlines() if line.startswith('PPid:')))
            except (OSError, ValueError, StopIteration):
                return False
        return False

    def environment(self, pid):
        return dict(item.decode().split('=', 1) for item in (self.proc / str(pid) / 'environ').read_bytes().split(b'\0') if b'=' in item)

    def retained_agent(self, expected=None, *, allow_missing=False):
        found=[]
        for path in self.proc.iterdir():
            if not path.name.isdecimal():
                continue
            try:
                argv=[x.decode() for x in (path/'cmdline').read_bytes().split(b'\0') if x]
                if not argv or Path(argv[0]).name!='MicroXRCEAgent':
                    continue
                pairs={key:argv[i+1] for i,key in enumerate(argv[:-1]) if key in ('-D','--dev','-b','--baudrate')}
                if 'serial' not in argv or pairs.get('-D',pairs.get('--dev'))!='/dev/ttyS2' or pairs.get('-b',pairs.get('--baudrate'))!='115200':
                    raise InstallError('发现不支持的既有 Agent 参数，未修改通信')
                found.append(self.process(int(path.name)))
            except (FileNotFoundError,ProcessLookupError):
                continue
        owners=subprocess.run(['fuser','/dev/ttyS2'],capture_output=True,text=True,timeout=4)
        if (allow_missing and not found and owners.returncode==1
                and not owners.stdout.strip() and not owners.stderr.strip()
                and Path('/dev/ttyS2').is_char_device()):
            return None
        if len(found)!=1 or found[0]['uid']!=self.uid:
            raise InstallError('必须复用本用户唯一 /dev/ttyS2 115200 Agent')
        if owners.returncode!=0 or not all(x.isdecimal() for x in owners.stdout.split()) or set(map(int,owners.stdout.split()))!={found[0]['pid']}:
            raise InstallError('无法确认原串口仅由已知 Agent 占用')
        if expected is not None and found[0]!=expected:
            raise InstallError('既有 Agent 身份变化，未切换/重启通信')
        return found[0]

    def boot_id(self):
        return (self.proc / 'sys/kernel/random/boot_id').read_text().strip()

    def live_state(self, controller):
        # Supervisor已经占用原生单客户端IPC；安装器不能抢第二连接，更不能让它断开。
        # 读现有原子快照，但必须核对boot/监督器/控制器及原生单调时间，不能给历史JSON续鲜。
        path=self.home/'FC01_navigation/status.json'
        deadline=time.monotonic()+5
        last=None
        while True:
            try:
                # systemctl可能需百毫秒：先取身份，再读最新快照，不能让慢查询耗尽快照TTL。
                # 仍用原始原生时间校验；不修改任何时间戳，也不延长.25s窗口。
                service=self.service()
                main=int(service.get('MainPID') or 0)
                stat_text=(self.proc/str(controller['pid'])/'stat').read_text()
                started=int(stat_text.rsplit(')',1)[1].split()[19])/os.sysconf('SC_CLK_TCK')
                boot=self.boot_id()
                if (path.is_symlink() or not path.is_file() or path.stat().st_uid!=self.uid
                        or path.stat().st_size>1048576):
                    raise InstallError('监督器状态文件来源或大小无效')
                row=json.loads(path.read_text())
                if (not main or row.get('supervisor_pid')!=main
                        or row.get('boot_id')!=boot
                        or row.get('controller_pid')!=controller['pid']
                        or row.get('state_controller_pid')!=controller['pid']
                        or row.get('status') not in
                           ('CONNECTED','PRESTREAM','READY_WAIT_ARM','WAIT_NAVIGATION','WAITING','FAULT','DIAGNOSTIC')
                        or row.get('feedback_current') is not True
                        or row.get('ipc_error') or row.get('controller_returncode') is not None
                        or row.get('retry_in_seconds') is not None):
                    raise InstallError('监督器/控制器/boot或实时IPC状态不匹配')
                state=row.get('state')
                # /proc stat第22列本次进程启动ticks继续绑定快照，防止PID复用。
                validate_snapshot_age(state,time.monotonic(),started)
                return state
            except (OSError,ValueError,TypeError,InstallError) as exc:
                last=exc
            if time.monotonic()>=deadline:
                raise InstallError('未取得新鲜且来源一致的监督器状态：'+str(last))
            self.sleep(.05)

    def dependencies(self, entry, manifest, environment):
        """先在目标Humble/SDK环境核对动态依赖和配置检查器，尚不启动控制器。"""
        # 包自带所需px4_msgs，本机旧SDK仅保留为来源信息，不能让旧typesupport抢先加载。
        script = '''set -eo pipefail
export PYTHONDONTWRITEBYTECODE=1
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH ROS_PACKAGE_PATH
source /opt/ros/humble/setup.bash
source "$1/install/local_setup.bash"
exec /usr/bin/python3 - "$1" <<'CHECK'
import subprocess,sys
from pathlib import Path
from rclpy.type_support import check_for_type_support
from px4_msgs.msg import VehicleStatus,VehicleLocalPosition,VehicleAttitude,ManualControlSwitches
from jikl_fc01_system.msg import VisionState,VisionHealth,NavigationCommand,VelocityCommand,AttitudeThrustPreview
for cls in (VehicleStatus,VehicleLocalPosition,VehicleAttitude,ManualControlSwitches,VisionState,VisionHealth,NavigationCommand,VelocityCommand,AttitudeThrustPreview):check_for_type_support(cls)
entry=Path(sys.argv[1])
for path in entry.rglob('*'):
 if not path.is_file():continue
 with path.open('rb') as stream:is_elf=stream.read(4)==b'\\x7fELF'
 if is_elf:
  result=subprocess.run(['ldd',str(path)],capture_output=True,text=True)
  report=result.stdout+result.stderr
  if result.returncode or 'not found' in report:raise SystemExit('unresolved runtime dependency '+str(path)+'\\n'+report)
print('FC01_PREBUILT_RUNTIME_DEPENDENCIES_OK')
CHECK
'''
        return self.run(['/bin/bash', '-c', script, 'fc01-prebuilt-check', str(entry)]).stdout

    def config_check(self, entry, manifest, runtime, environment):
        binary = entry / manifest['controller_relative']
        checker = binary.with_name('fc01_config_check')
        if not checker.is_file() or not verify_elf(checker):
            raise InstallError('缺少 AArch64 配置检查器')
        self.run(['/bin/bash', str(entry/'prebuilt_boot.sh'), 'check', '--config', str(runtime)])

    def systemd_verify(self, unit):
        # 验证合成完整unit：drop-in自身没有[Unit]/[Install]，不把它误作独立服务运行。
        self.run(['/usr/bin/systemd-analyze', 'verify', str(unit)])

    def write_root(self, path, data, mode=0o644):
        with tempfile.NamedTemporaryFile() as source:
            source.write(data)
            source.flush()
            self.privileged(['/usr/bin/install', '-D', '-m', format(mode, '04o'), source.name, str(path)])

    def remove_root(self, path):
        self.privileged(['/usr/bin/rm', '-f', '--', str(path)])

    def reload(self):
        self.privileged(['/usr/bin/systemctl', 'daemon-reload'])

    def stop(self):
        self.privileged(['/usr/bin/systemctl', 'stop', '--no-block', SERVICE])

    def start(self):
        self.privileged(['/usr/bin/systemctl', 'start', SERVICE])

    def sleep(self, seconds):
        time.sleep(seconds)


def validate_snapshot_age(state, now, started):
    """将快照留存时间计入各源TTL；原生时间戳必须属于本次进程，短轮询等待新快照。"""
    if not isinstance(state,dict):raise InstallError('缺少原生状态')
    stamp=state.get('diagnostic_monotonic_s')
    if type(stamp) not in (int,float) or not math.isfinite(stamp):
        raise InstallError('缺少原生单调采样时间')
    elapsed=now-stamp
    if stamp<started or not 0<=elapsed<=.25:
        raise InstallError('原生状态已旧或不属于本次进程')
    timings=state.get('source_timing')
    if not isinstance(timings,dict):raise InstallError('缺少反馈时间证据')
    for topic in ('status','land','local','attitude','flags','battery'):
        item=timings.get(topic)
        if not isinstance(item,dict):raise InstallError('缺少源时间证据：'+topic)
        timeout=item.get('timeout_s')
        if (type(timeout) not in (int,float) or not math.isfinite(timeout) or timeout<=0
                or item.get('source_reason')!='VALID' or item.get('receipt_reason')!='VALID'):
            raise InstallError('反馈源时钟或接收时间无效：'+topic)
        names=('envelope_age_s','receipt_age_s')
        if topic in ('local','attitude'):names+=('sample_age_s','sample_progress_age_s')
        for name in names:
            age=item.get(name)
            if (type(age) not in (int,float) or not math.isfinite(age)
                    or (name in ('receipt_age_s','sample_progress_age_s') and age<0)
                    or max(0,age)+elapsed>=timeout):
                raise InstallError('快照中的反馈已过期：'+topic+'/'+name)


def require_ground(backend, service=None):
    """从现存控制器即时STATUS确认未解锁接地，不能给旧JSON快照续鲜。"""
    service = service or backend.service()
    main = int(service.get('MainPID') or 0)
    controllers = backend.controllers()
    if service.get('ActiveState') != 'active' or not main or len(controllers) != 1:
        raise InstallError('安装/回退需当前受管服务 active 且恰好一个控制器')
    controller = controllers[0]
    if controller['uid'] != backend.uid or not backend.descendant(controller['pid'], main):
        raise InstallError('控制器不属于当前服务/用户')
    state = backend.live_state(controller)
    thrust=state.get('thrust_body')
    if (not isinstance(thrust,list) or len(thrust)!=3
            or any(type(v) not in (int,float) or not math.isfinite(v) or v!=0 for v in thrust)
            or state.get('controller_pid') != controller['pid'] or state.get('armed') is not False
            or state.get('nav_state')==14
            or any(state.get(name) is not True for name in
                   ('arming_state_known', 'status_fresh', 'land_fresh', 'local_fresh',
                    'attitude_fresh', 'source_clock_valid', 'landed', 'at_rest'))
            ):
        raise InstallError('尚无当前控制器新鲜未解锁、接地、静止、零推力证据')
    # 人工解锁前的RC预流本来就active且发送零推力心跳，不把这种正常等待误作飞行任务。
    if ((state.get('active') is not False or state.get('heartbeat_enabled') is not False)
            and state.get('phase') not in ('RC_PRESTREAM','RC_READY_WAIT_ARM','RC_ARMED_WAIT_NAVIGATION')):
        raise InstallError('仍处于非准备任务阶段，未切换服务')
    if state.get('rc_navigation_enabled'):
        if (state.get('rc_axes_safe') is not True or state.get('rc_arm_switch') != 3
                or state.get('rc_kill_switch') != 3
                or type(state.get('rc_mode_slot')) is not int or not 1<=state['rc_mode_slot']<=6
                or state.get('rc_mode_slot') == state.get('rc_navigation_mode_slot')
                or any(state.get(key) is not True for key in
                       ('rc_source_valid','rc_switches_fresh','rc_link_ready','rc_switches_ready',
                        'rc_source_clock_valid','rc_switch_clock_valid'))):
            raise InstallError('RC未处于新鲜安全地面开关状态')
    return controller, state


def capture_environment(backend, service, controller):
    main = int(service['MainPID'])
    env = backend.environment(main)
    env.update(backend.environment(controller['pid']))
    inherited = {name: env[name] for name in ENV_KEYS if name in env}
    if inherited.get('ROS_DOMAIN_ID', '0') != '0' or inherited.get('ROS_LOCALHOST_ONLY', '0') != '0':
        raise InstallError('现行非域0/非网络DDS不符合既有服务合同；未悄悄改设置')
    sdk = env.get('FC01_PROJECT_ROOT')
    if sdk and not Path(sdk).is_absolute():
        raise InstallError('实际服务 SDK 路径不是绝对路径')
    cwd = service.get('WorkingDirectory')
    if not cwd or not Path(cwd).is_absolute():
        raise InstallError('原服务工作目录未知')
    return dict(environment=inherited, sdk_root=sdk, original_entry=cwd,
                service_mode=None, controller_environment_sha256=hashlib.sha256(json.dumps(env, sort_keys=True).encode()).hexdigest())


def agent_policy(backend,agent,service):
    """只有原监督器自建且参数与既有创建合同完全一致，才允许停旧后由新监督器重建。"""
    owned=backend.descendant(agent['pid'],int(service['MainPID']))
    expected=['serial','-D','/dev/ttyS2','-b','115200','-v','0']
    if owned and agent['argv'][1:]!=expected:
        raise InstallError('原监督器自有Agent参数不符合现有创建合同，未停止通信')
    return owned


def verify_agent(backend,original,owned,*,allow_missing=False):
    """共享Agent逐项保持身份；自有Agent仅在已停控制器且UART空闲时允许缺席或PID交接。"""
    missing=allow_missing and owned and not backend.controllers()
    current=backend.retained_agent(allow_missing=missing)
    if current is None:
        return None
    if owned:
        if any(current.get(k)!=original.get(k) for k in ('uid','exe','sha256')) or current['argv'][1:]!=original['argv'][1:]:
            raise InstallError('交接后的Agent程序/摘要/通信参数变化')
    elif current!=original:
        raise InstallError('共享Agent身份变化，未重启或覆盖通信')
    return current


def current_runtime(controller):
    values = [controller['argv'][i+1] for i,arg in enumerate(controller['argv'][:-1]) if arg == '--params-file']
    if len(values) != 1:
        raise InstallError('无法确认唯一生效的 --params-file')
    path = Path(values[0])
    if not path.is_absolute() or path.is_symlink() or not path.is_file():
        raise InstallError('生效参数文件无效或为符号链接')
    import yaml
    config = yaml.safe_load(path.read_text())
    try:
        params = config['fc01_system']['ros__parameters']
    except (KeyError, TypeError) as exc:
        raise InstallError('生效参数结构不完整') from exc
    if (not isinstance(params, dict) or params.get('visual_navigation_enabled') is not False
            or params.get('arm_enabled') is not False):
        raise InstallError('只继承视觉关闭、人工RC解锁合同，不扩大权限')
    mode = params.get('fc01_runtime_mode')
    if mode not in ('diagnostic', 'flight-test'):
        raise InstallError('无法确认原模式 diagnostic/flight-test')
    return path, config, mode


def verify_effective_policy(state,config):
    """磁盘runtime可能在进程启动后被改；可观测的控制门/RC/目标必须与实际节点一致。"""
    params=config['fc01_system']['ros__parameters']
    for key in ('profile','rc_target_height','rc_navigation_enabled','rc_navigation_mode_slot',
                'flight_enabled','flight_actions_enabled','arm_enabled','takeoff_enabled',
                'flight_parameters_confirmed','bench_arm_only_enabled','visual_navigation_enabled',
                'rc_takeover_confirmed','autonomous_failsafe_confirmed'):
        if key not in state or key not in params or state[key]!=params[key]:
            raise InstallError('运行节点目标/RC/动作门与磁盘生效参数不一致，未切换：'+key)


def capture_files(backend, service):
    unit = Path(service.get('FragmentPath', ''))
    if unit != backend.units / SERVICE or unit.is_symlink() or not unit.is_file() or not unit.read_text().startswith(MARKER+'\n'):
        raise InstallError('当前服务不是已知受管FC01单元，未覆盖')
    if service.get('User') not in (backend.user, str(backend.uid)):
        raise InstallError('当前服务用户与安装用户不一致')
    paths = [unit]
    paths += [Path(p) for p in shlex.split(service.get('DropInPaths', ''))]
    # 环境文件绝不能被新版本路径替换悄悄遗失；先保留实际字节，且拒绝未知/带链接来源。
    envfiles = service.get('EnvironmentFiles', '')
    for name in re.findall(r'(/[^ ;{}]+)(?:\s+\(ignore_errors=[^)]*\))?', envfiles):
        paths.append(Path(name))
    result = []
    for path in paths:
        if (not path.is_absolute() or any(p.is_symlink() for p in (path,*path.parents))
                or not path.is_file()):
            raise InstallError('服务配置来源无效：' + str(path))
        data = path.read_bytes()
        result.append(dict(path=str(path), mode=path.stat().st_mode & 0o777,
                           sha256=hashlib.sha256(data).hexdigest(), data=data))
    own = backend.units / (SERVICE + '.d') / DROPIN
    if any(p.is_symlink() for p in (own,*own.parents)):
        raise InstallError('事务drop-in路径包含链接，未覆盖')
    if own not in [Path(item['path']) for item in result]:
        result.append(dict(path=str(own), present=False, data=None))
    return result


def snapshot(backend, entry, archive_hash, service, controller, runtime, environment, files, agent,agent_owned):
    folder = backend.home / 'FC01_navigation/prebuilt-backups'
    safe_directory(folder, backend.uid)
    folder.mkdir(mode=0o700, parents=True, exist_ok=True)
    backup = folder / (datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ') + '-' + str(os.getpid()))
    backup.mkdir(mode=0o700)
    records=[]
    for i,item in enumerate(files):
        row={key:value for key,value in item.items() if key!='data'}
        if item.get('data') is not None:
            name=f'file-{i:03d}'
            (backup/name).write_bytes(item['data']);(backup/name).chmod(0o600)
            row.update(present=True, backup=name)
        records.append(row)
    original_runtime = Path(runtime)
    (backup/'runtime.yaml').write_bytes(original_runtime.read_bytes())
    manifest=dict(schema=1,entry=str(entry),archive_sha256=archive_hash,service=service,
                  controller=controller,runtime_path=str(original_runtime),
                  runtime_sha256=digest(original_runtime),runtime_mode=original_runtime.stat().st_mode&0o777,
                  environment=environment,files=records,agent=agent,agent_owned=agent_owned,
                  boot_id=backend.boot_id(),phase='prepared')
    atomic_json(backup/'BACKUP.json',manifest)
    return backup,manifest


def write_deployment(entry, config, mode, environment, manifest, backup):
    import yaml
    folder=entry/'deployment'
    if folder.is_symlink():raise InstallError('deployment目录被改为链接，未覆盖')
    folder.mkdir(mode=0o700,exist_ok=True)
    for name in ('runtime.yaml','environment.json','INSTALL_STATE.json'):
        if (folder/name).is_symlink():raise InstallError('本机配置为链接，未覆盖：'+name)
    params=config['fc01_system']['ros__parameters']
    params['fc01_config_path']=str(entry.parents[1]/'FC01_navigation/runtime.yaml')
    params['fc01_config_source_path']=str(folder/'runtime.yaml')
    params['fc01_entry_version']=manifest['version']
    data=yaml.safe_dump(config,sort_keys=False,allow_unicode=True)
    (folder/'runtime.yaml').write_text(data);(folder/'runtime.yaml').chmod(0o600)
    environment=dict(environment, service_mode=mode, mode=mode)
    atomic_json(folder/'environment.json',environment)
    atomic_json(folder/'INSTALL_STATE.json',dict(backup=str(backup),runtime_sha256=digest(folder/'runtime.yaml'),phase='prepared'))


def stop_and_wait(backend, *, timeout=30):
    info=backend.service()
    controllers=backend.controllers()
    if info.get('ActiveState') in ('inactive','failed') and int(info.get('MainPID') or 0)==0 and not controllers:
        return
    # 启动失败可能只剩Supervisor、尚无控制节点；此时仍应正常停服务再恢复，不能强杀。
    # 只要仍有控制器，就必须重新取得它自己的实时未解锁地面证据。
    if controllers:
        require_ground(backend)
    elif info.get('User') not in (backend.user,str(backend.uid)):
        raise InstallError('无控制器但服务用户未知，未停止它')
    backend.stop()
    deadline=time.monotonic()+timeout
    while True:
        info=backend.service()
        if (info.get('ActiveState') in ('inactive','failed') and int(info.get('MainPID') or 0)==0
                and not backend.controllers()):
            return
        if time.monotonic()>=deadline:
            raise InstallError('控制器尚未安全退出，未强杀、未改写运行配置')
        backend.sleep(.1)


def verify_active(backend, entry, manifest, environment, *, timeout=STARTUP_ACCEPTANCE_SECONDS):
    deadline=time.monotonic()+timeout
    last=None
    while True:
        try:
            info=backend.service()
            controller,state=require_ground(backend,info)
            binary=entry/manifest['controller_relative']
            if (Path(controller['exe'])!=binary.resolve() or controller['sha256']!=digest(binary)
                    or Path(controller['cwd'])!=entry.resolve()
                    or str(entry/'prebuilt_boot.sh') not in info.get('ExecStart','')
                    or Path(info.get('WorkingDirectory','/'))!=entry.resolve()):
                raise InstallError('实际服务/控制器路径或摘要未指向候选版本')
            actual_env=backend.environment(controller['pid'])
            if any(actual_env.get(k)!=v for k,v in environment['environment'].items()):
                raise InstallError('实际DDS环境未继承')
            config_path=entry/'deployment/runtime.yaml'
            expected_config=backend.home/'FC01_navigation/runtime.yaml'
            if state.get('fc01_config_path',state.get('config_path',str(expected_config)))!=str(expected_config):
                raise InstallError('实际原生配置路径错误')
            live_path,config,_=current_runtime(controller)
            verify_effective_policy(state,config)
            if config!=__import__('yaml').safe_load(config_path.read_text()):
                raise InstallError('实际运行参数与继承快照不同')
            if live_path!=backend.home/'FC01_navigation/runtime.yaml' and live_path!=config_path:
                raise InstallError('实际控制器使用未知参数路径')
            return dict(controller=controller,state=state,service=info)
        except (OSError,ValueError,InstallError,subprocess.SubprocessError) as exc:
            last=exc
        if time.monotonic()>=deadline:
            raise InstallError('新版启动验收失败：'+str(last))
        backend.sleep(.1)


def assert_source_files_unchanged(files):
    for item in files:
        path=Path(item['path'])
        current=digest(path) if path.is_file() and not path.is_symlink() else None
        if item.get('present',True) and current!=item['sha256']:
            raise InstallError('原服务配置被外部修改，未覆盖：'+str(path))
        if not item.get('present',True) and path.exists():
            raise InstallError('事务drop-in被外部创建，未覆盖')


def validate_restore(backend, backup, record):
    """先完整验证回退来源；损坏备份或外部编辑不能先停掉正常运行的候选版本。"""
    own=backend.units/(SERVICE+'.d')/DROPIN
    paths=[item.get('path') for item in record.get('files',[])]
    if (len(paths)!=len(set(paths)) or paths.count(str(own))!=1
            or paths.count(str(backend.units/SERVICE))!=1):
        raise InstallError('回退服务文件清单无效')
    for item in record['files']:
        path=Path(item['path'])
        if path==own:
            continue
        if path.is_symlink() or not path.is_file() or digest(path)!=item.get('sha256'):
            raise InstallError('原服务配置被外部修改，保留它：'+str(path))
    # 备份必须全部验证后才开始恢复，不能恢复一半才发现另一项损坏。
    for item in record['files']:
        if item.get('present'):
            p=backup/item['backup']
            if (not re.fullmatch(r'file-[0-9]{3}',item['backup']) or p.is_symlink()
                    or digest(p)!=item['sha256']):
                raise InstallError('服务备份摘要损坏')
    runtime=backup/'runtime.yaml'
    if runtime.is_symlink() or digest(runtime)!=record['runtime_sha256']:
        raise InstallError('参数备份摘要损坏')
    live=Path(record['runtime_path'])
    if (not live.is_absolute() or not live.is_relative_to(backend.home)
            or any(p.is_symlink() for p in (live,*live.parents))
            or not live.is_file() or live.stat().st_uid!=backend.uid):
        raise InstallError('原参数恢复路径不在本用户可验证的普通文件范围')


def restore(backend, backup, record, *, restart=True):
    if backend.controllers():
        raise InstallError('仍有控制器，不能在其运行期间恢复配置')
    verify_agent(backend,record['agent'],record.get('agent_owned',False),allow_missing=True)
    validate_restore(backend,backup,record)
    own=backend.units/(SERVICE+'.d')/DROPIN
    # 主unit、原drop-ins和环境文件从未改写，且上面已确认原字节仍完整；仅恢复本事务的drop-in。
    # 不以用户可写备份清单中的任意路径执行root写入。
    for item in record['files']:
        path=Path(item['path'])
        if path!=own:
            continue
        if item.get('present'):
            backend.write_root(path,(backup/item['backup']).read_bytes(),item['mode'])
        else:
            backend.remove_root(path)
    runtime=backup/'runtime.yaml'
    live=Path(record['runtime_path'])
    if live.is_symlink():raise InstallError('运行参数被改为链接，保留它')
    temp=live.with_name('.'+live.name+'.prebuilt-restore')
    temp.write_bytes(runtime.read_bytes());temp.chmod(record['runtime_mode']);os.replace(temp,live)
    backend.reload()
    # 本事务没有改enable状态；主unit/drop-ins原样还原，不需重新解释开机策略。
    if restart and record['service'].get('ActiveState')=='active':
        backend.start()
        verify_original(backend,backup,record)


def verify_original(backend,backup,record,*,timeout=30):
    """start返回不代表恢复成功；核对旧实际二进制、完整生效参数和原DDS设置。"""
    import yaml
    expected=yaml.safe_load((backup/'runtime.yaml').read_text())
    deadline=time.monotonic()+timeout
    while True:
        try:
            controller,state=require_ground(backend)
            old=record['controller']
            if controller['exe']!=old['exe'] or controller['sha256']!=old['sha256']:
                raise InstallError('回退运行控制器未匹配原版')
            _,config,_=current_runtime(controller)
            verify_effective_policy(state,config)
            if config!=expected:raise InstallError('回退后实际运行参数与原始备份不同')
            actual=backend.environment(controller['pid'])
            if any(actual.get(k)!=v for k,v in record['environment']['environment'].items()):
                raise InstallError('回退后实际DDS设置与原始备份不同')
            verify_agent(backend,record['agent'],record.get('agent_owned',False))
            return
        except (OSError,ValueError,InstallError,subprocess.SubprocessError) as exc:
            last=exc
        if time.monotonic()>=deadline:
            raise InstallError('原配置已恢复，但原服务启动/参数/地面验收失败；需人工核对：'+str(last))
        backend.sleep(.1)


def install(archive, expected, backend):
    backend.check_host()
    folder=backend.home/'FC01_versions'
    safe_directory(folder,backend.uid);folder.mkdir(mode=0o700,parents=True,exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='.fc01-unpack-',dir=folder) as tmp:
        extracted,manifest=extract_checked(archive,expected,Path(tmp))
        entry=folder/manifest['version']
        safe_directory(entry,backend.uid)
        if entry.exists():
            saved=json.loads((entry/MANIFEST).read_text())
            if saved!=manifest:raise InstallError('同名版本目录属于不同发布，未覆盖')
            verify_tree(entry,manifest)
        else:
            publish_directory(extracted,entry)
        service=backend.service()
        controller,state=require_ground(backend,service)
        agent=backend.retained_agent()
        owned=agent_policy(backend,agent,service)
        environment=capture_environment(backend,service,controller)
        environment['agent_path']=agent['exe']
        runtime,config,mode=current_runtime(controller)
        environment['service_mode']=mode
        # 已激活重复安装仍真实验收，不停服务、不重写继承配置，不覆盖原回退点。
        installed=entry/'deployment/INSTALL_STATE.json'
        if Path(controller['exe'])==(entry/manifest['controller_relative']).resolve():
            if not installed.is_file():raise InstallError('已有本版进程但缺少安装事务记录')
            row=json.loads(installed.read_text())
            env=json.loads((entry/'deployment/environment.json').read_text())
            backend.dependencies(entry,manifest,env)
            verify_active(backend,entry,manifest,env)
            verify_agent(backend,agent,owned)
            if digest(entry/'deployment/runtime.yaml')!=row['runtime_sha256']:
                raise InstallError('继承参数被修改，重复安装不覆盖')
            return dict(entry=str(entry),backup=row['backup'],already_active=True,version=manifest['version'])
        files=capture_files(backend,service)
        verify_effective_policy(state,config)
        backend.dependencies(entry,manifest,environment)
        backend.authenticate()
        # sudo可能等待；重新核对实时地面和旧controller，不拿旧证据授权。
        current,_=require_ground(backend)
        if current!=controller:raise InstallError('认证期间旧控制器已变化')
        verify_agent(backend,agent,owned)
        assert_source_files_unchanged(files)
        backup,record=snapshot(backend,entry,expected,service,controller,runtime,environment,files,agent,owned)
        write_deployment(entry,config,mode,environment,manifest,backup)
        backend.config_check(entry,manifest,entry/'deployment/runtime.yaml',environment)
        own=backend.units/(SERVICE+'.d')/DROPIN
        dropin=('[Service]\nExecStart=\nExecStart=/bin/bash '+systemd_quote(entry/'prebuilt_boot.sh')+'\nWorkingDirectory='+systemd_directory(entry)+'\n').encode()
        with tempfile.TemporaryDirectory(prefix='fc01-unit-check-') as unit_tmp:
            unit=Path(unit_tmp)/SERVICE
            unit.write_bytes(Path(record['service']['FragmentPath']).read_bytes()+b'\n'+dropin)
            backend.systemd_verify(unit)
        changed=started=False
        try:
            stop_and_wait(backend)
            verify_agent(backend,agent,owned,allow_missing=True)
            # 在停旧前后都确认原参数未被外部编辑；不覆盖他人刚完成的调整。
            if digest(runtime)!=record['runtime_sha256']:
                raise InstallError('原生效参数在安装期间被修改，未切换')
            assert_source_files_unchanged(files)
            changed=True
            backend.write_root(own,dropin)
            backend.reload()
            atomic_json(entry/'deployment/INSTALL_STATE.json',dict(backup=str(backup),runtime_sha256=digest(entry/'deployment/runtime.yaml'),phase='starting',dropin_sha256=hashlib.sha256(dropin).hexdigest()))
            started=True
            backend.start()
            verify_active(backend,entry,manifest,environment)
            verify_agent(backend,agent,owned)
            record.update(phase='active',installed_dropin_sha256=hashlib.sha256(dropin).hexdigest())
            atomic_json(backup/'BACKUP.json',record)
            atomic_json(entry/'deployment/INSTALL_STATE.json',dict(backup=str(backup),runtime_sha256=digest(entry/'deployment/runtime.yaml'),phase='active',dropin_sha256=record['installed_dropin_sha256']))
            return dict(entry=str(entry),backup=str(backup),already_active=False,version=manifest['version'])
        except BaseException as exc:
            try:
                if started:
                    stop_and_wait(backend)
                if changed:
                    if own.is_symlink() or (own.exists() and digest(own)!=hashlib.sha256(dropin).hexdigest()):
                        raise InstallError('新drop-in被外部修改，未覆盖')
                    restore(backend,backup,record)
                elif not backend.controllers() and service.get('ActiveState')=='active':
                    backend.start()
                    verify_original(backend,backup,record)
                record.update(phase='failed_restored',error=repr(exc));atomic_json(backup/'BACKUP.json',record)
            except BaseException as recovery:
                record.update(phase='manual_recovery_required',error=repr(exc),recovery_error=repr(recovery));atomic_json(backup/'BACKUP.json',record)
                raise InstallError('安装失败且安全恢复未完成；未强杀。备份='+str(backup)+'；'+str(recovery)) from exc
            raise InstallError('安装失败，原版配置和运行状态已恢复。备份='+str(backup)+'；'+str(exc)) from exc


def rollback(backup,backend):
    backend.check_host()
    backup=Path(backup).resolve()
    allowed=(backend.home/'FC01_navigation/prebuilt-backups').resolve()
    if not backup.is_relative_to(allowed) or backup==allowed:
        raise InstallError('回退目录不在本用户预编译备份范围')
    record=json.loads((backup/'BACKUP.json').read_text())
    entry=Path(record['entry'])
    if not entry.is_relative_to(backend.home/'FC01_versions'):
        raise InstallError('备份版本目录无效')
    own=backend.units/(SERVICE+'.d')/DROPIN
    if own.is_symlink() or not own.is_file() or digest(own)!=record.get('installed_dropin_sha256'):
        raise InstallError('当前服务切换记录不匹配，拒绝覆盖')
    installed=json.loads((entry/'deployment/INSTALL_STATE.json').read_text())
    if Path(installed.get('backup','')).resolve()!=backup:
        raise InstallError('候选版本与回退目录不匹配')
    validate_restore(backend,backup,record)
    backend.authenticate()
    release=json.loads((entry/MANIFEST).read_text())
    info=backend.service()
    controllers=backend.controllers()
    if not controllers and info.get('ActiveState') in ('inactive','failed') and int(info.get('MainPID') or 0)==0:
        verify_agent(backend,record['agent'],record.get('agent_owned',False),allow_missing=True)
    else:
        current,_=require_ground(backend)
        if Path(current['exe'])!=(entry/release['controller_relative']).resolve():
            raise InstallError('当前运行版本不是本回退记录的候选，未停止它')
    # sudo交互期间备份/服务源仍可能变化；停止前再次验证，恢复前还会再验一次。
    validate_restore(backend,backup,record)
    stop_and_wait(backend)
    restore(backend,backup,record)
    record['phase']='rolled_back';atomic_json(backup/'BACKUP.json',record)
    return dict(backup=str(backup),restored_controller=record['controller']['exe'])


def status(backend):
    service=backend.service()
    controllers=backend.controllers()
    result=dict(service=service,controllers=controllers)
    if len(controllers)==1:
        result['state']=backend.live_state(controllers[0])
    return result


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('archive',type=Path,nargs='?')
    parser.add_argument('--expected-sha256')
    parser.add_argument('--rollback',type=Path)
    parser.add_argument('--status',action='store_true')
    args=parser.parse_args(argv)
    try:
        backend=Backend()
        if args.status:
            print(json.dumps(status(backend),ensure_ascii=False,indent=2));return 0
        if bool(args.rollback)==bool(args.archive):
            parser.error('指定安装包或 --rollback，不能同时提供')
        lock=backend.home/'.cache/fc01_navigation/prebuilt-install.lock'
        lock.parent.mkdir(mode=0o700,parents=True,exist_ok=True)
        with lock.open('a') as stream:
            fcntl.flock(stream,fcntl.LOCK_EX|fcntl.LOCK_NB)
            result=rollback(args.rollback,backend) if args.rollback else install(args.archive,args.expected_sha256,backend)
        print(json.dumps(result,ensure_ascii=False,indent=2))
        if 'backup' in result:
            print('FC01_ROLLBACK_COMMAND=bash ./install_fc01.sh --rollback '+shlex.quote(result['backup']))
        return 0
    except (OSError,ValueError,TypeError,KeyError,RuntimeError,subprocess.SubprocessError) as exc:
        print('FC01_PREBUILT_INSTALL_FAILED: '+str(exc),file=sys.stderr)
        return 1


if __name__=='__main__':
    raise SystemExit(main())
