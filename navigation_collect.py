#!/usr/bin/env python3
# 现有部署/通信身份的只读采集器：默认仅标准库；可选ROS子进程只订阅/fmu/out且有时间上限。
# CDR大小不是UART吞吐，source age未验证同步时只作观察；磁盘源码不证明运行二进制实际用了它。
"""Collect existing navigation deployment evidence; never start services or publish.

The default path needs only Python's standard library. ROS sampling is opt-in,
uses subscriptions to /fmu/out only, and runs in a child with a wall-clock limit.
"""
from __future__ import annotations
import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import shlex
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

# 只保留与HOME/SDK/ROS相关环境白名单，避免复制服务的其它凭据。
ENV_KEYS = (
    'HOME',
    'FC01_PROJECT_ROOT',
    'ROS_DOMAIN_ID',
    'ROS_LOCALHOST_ONLY',
    'RMW_IMPLEMENTATION',
    'AMENT_PREFIX_PATH',
    'CMAKE_PREFIX_PATH',
    'COLCON_PREFIX_PATH',
    'LD_LIBRARY_PATH',
    'PYTHONPATH',
    'ROS_DISTRO',
)
SERVICE_KEYS = (
    'WorkingDirectory',
    'ExecStart',
    'Environment',
    'EnvironmentFiles',
    'LoadState',
    'ActiveState',
    'SubState',
    'MainPID',
    'User',
    'FragmentPath',
    'DropInPaths',
    'UnitFileState',
    'ControlGroup',
)
# 采样话题均为PX4输出，不包含/fmu/in控制入口；每个观察订阅也会增加DDS负载。
TOPICS = {
    '/fmu/out/vehicle_attitude': 'VehicleAttitude',
    '/fmu/out/vehicle_local_position': 'VehicleLocalPosition',
    '/fmu/out/vehicle_status_v1': 'VehicleStatus',
    '/fmu/out/vehicle_land_detected': 'VehicleLandDetected',
    '/fmu/out/battery_status': 'BatteryStatus',
    '/fmu/out/failsafe_flags': 'FailsafeFlags',
    '/fmu/out/manual_control_setpoint': 'ManualControlSetpoint',
    '/fmu/out/manual_control_switches': 'ManualControlSwitches',
    '/fmu/out/vehicle_command_ack': 'VehicleCommandAck',
}


# 有界执行只读查询并裁剪输出；超时/权限/非零都报告unknown，不假设不存在。
def run_command(argv, timeout=4):
    try:
        result = subprocess.run(
            argv,
            stdin=subprocess.DEVNULL,
            capture_output=True,
            text=True,
            errors='replace',
            timeout=timeout,
            check=False,
        )
        return dict(
            status='ok' if result.returncode == 0 else 'unknown',
            returncode=result.returncode,
            stdout=result.stdout[:131072],
            stderr=result.stderr[:4096],
            truncated=len(result.stdout) > 131072,
        )
    except subprocess.TimeoutExpired:
        return dict(status='unknown', reason=f'timeout after {timeout}s')
    except OSError as error:
        return dict(status='unknown', reason=str(error))


# 限制单文件256MiB，读前后大小/mtime不变才给摘要；并发改写返回unknown。
def fingerprint(path):
    path = Path(path)
    try:
        before = path.stat()
        if not path.is_file() or before.st_size > 256 * 1024 * 1024:
            raise OSError('not a regular file or exceeds 256 MiB hash limit')
        digest = hashlib.sha256()
        with path.open('rb') as stream:
            for block in iter(lambda: stream.read(1024 * 1024), b''):
                digest.update(block)
        after = path.stat()
        if (before.st_size, before.st_mtime_ns) != (after.st_size, after.st_mtime_ns):
            raise OSError('file changed while hashing')
        return dict(
            path=str(path),
            status='ok',
            sha256=digest.hexdigest(),
            size=after.st_size,
            mtime_ns=after.st_mtime_ns,
        )
    except OSError as error:
        return dict(path=str(path), status='unknown', reason=str(error))


# 分别解析systemd shell词或/proc空字符环境，只返回白名单键。
def environment(text, separator=None):
    try:
        words = text.split(separator) if separator else shlex.split(text)
        return {
            key: value
            for word in words
            if '=' in word
            for key, value in [word.split('=', 1)]
            if key in ENV_KEYS
        }
    except ValueError:
        return {}


# 只观察主PID/Agent/控制器与监督器，保留exe/cwd/串口fd身份；不可读进程计入partial。
def processes(proc_root, main_pid):
    items = []
    unreadable = 0
    try:
        entries = list(Path(proc_root).iterdir())
    except OSError as error:
        return dict(status='unknown', reason=str(error), items=[])
    for entry in entries:
        if not entry.name.isdecimal():
            continue
        try:
            argv = [
                x
                for x in (entry / 'cmdline').read_bytes().decode(errors='replace').split('\0')
                if x
            ]
            comm = (entry / 'comm').read_text().strip()
            if (
                entry.name != main_pid
                and comm not in ('MicroXRCEAgent', 'fc01_system_node')
                and not any(
                    Path(x).name
                    in ('MicroXRCEAgent', 'fc01_system_node', 'navigation_service.py', 'standby.py')
                    for x in argv[:3]
                )
            ):
                continue
            row = dict(pid=int(entry.name), argv=argv, comm=comm, uid=entry.stat().st_uid)
            try:
                row['environment'] = environment(
                    (entry / 'environ').read_bytes().decode(errors='replace'), '\0'
                )
            except OSError as error:
                row['environment'] = {}
                row['environment_error'] = str(error)
            row['executable'] = fingerprint(entry / 'exe')
            for name in ('exe', 'cwd'):
                try:
                    row[name] = os.readlink(entry / name)
                except OSError:
                    row[name] = None
            row['serial_fds'] = {}
            try:
                for fd in (entry / 'fd').iterdir():
                    try:
                        target = os.readlink(fd)
                        if target.startswith(('/dev/tty', '/dev/serial/')):
                            row['serial_fds'][fd.name] = target
                    except OSError:
                        pass
            except OSError as error:
                row['serial_fds_error'] = str(error)
            items.append(row)
        except OSError:
            unreadable += 1
    return dict(
        status='partial' if unreadable else 'ok',
        unreadable_processes=unreadable,
        items=sorted(items, key=lambda row: row['pid']),
    )


# 以服务WorkingDirectory为首选入口，收源码/参数/SDK与进程身份；不source或执行部署脚本。
def collect(service, device, project_root=None, command=run_command, proc_root=Path('/proc')):
    shown = command(
        ['systemctl', 'show', '--no-pager', service, '--property=' + ','.join(SERVICE_KEYS)]
    )
    properties = dict(
        line.split('=', 1) for line in shown.get('stdout', '').splitlines() if '=' in line
    )
    service_env = environment(properties.get('Environment', ''))
    # Keep the relevant environment, never copy unrelated service credentials.
    if 'Environment' in properties:
        properties['Environment'] = service_env
    service_info = dict(
        status=shown['status'],
        properties=properties,
        environment_scope='Only navigation/ROS/SDK keys are retained.',
    )
    for key in ('reason', 'returncode', 'stderr', 'truncated'):
        if key in shown:
            service_info[key] = shown[key]
    if properties.get('LoadState') in ('not-found', 'error', 'bad-setting'):
        service_info.update(status='unknown', reason='unit LoadState=' + properties['LoadState'])
    process_info = processes(proc_root, properties.get('MainPID', '0'))
    entry = Path(properties.get('WorkingDirectory') or Path(__file__).resolve().parent)
    root_value = (
        project_root or service_env.get('FC01_PROJECT_ROOT') or os.environ.get('FC01_PROJECT_ROOT')
    )
    project = Path(root_value).expanduser() if root_value else None
    paths = {
        entry / name
        for name in (
            'navigation_boot.sh',
            'navigation_service.py',
            'standby.py',
            'launcher.py',
            'navigation_settings.json',
            'vision_settings.json',
            'runtime.yaml',
        )
    }
    paths.update(entry.glob('*.py'))
    paths.update(entry.glob('*.sh'))
    home = Path(service_env.get('HOME') or Path.home())
    paths.add(home / 'FC01_navigation/runtime.yaml')
    paths.add(entry / 'install/lib/jikl_fc01_system/fc01_system_node')
    for key in ('FragmentPath', 'DropInPaths'):
        try:
            paths.update(Path(x) for x in shlex.split(properties.get(key, '')) if x.startswith('/'))
        except ValueError:
            pass
    # 源码摘要最多1000项并明确记录上限是否到达，不能将有界采集说成全树取证。
    source_count = 0
    try:
        for source in (entry / 'native/src').rglob('*'):
            if source.is_file() and source.suffix in (
                '.cpp',
                '.hpp',
                '.h',
                '.yaml',
                '.xml',
                '.txt',
            ):
                if source_count >= 1000:
                    break
                paths.add(source)
                source_count += 1
    except OSError:
        pass
    sdk_prefixes = set()
    environments = [service_env, {k: os.environ[k] for k in ENV_KEYS if k in os.environ}]
    for row in process_info['items']:
        environments.append(row['environment'])
        argv = row['argv']
        for index, word in enumerate(argv):
            value = argv[index + 1] if word == '--params-file' and index + 1 < len(argv) else None
            if word.startswith('--params-file='):
                value = word.split('=', 1)[1]
            if value:
                path = Path(value)
                paths.add(path if path.is_absolute() else Path(row.get('cwd') or entry) / path)
    if project:
        paths.add(project / 'config/attitude_altitude.yaml')
        sdk_prefixes.add(project / 'install_r632')
    for env in environments:
        for key in ('AMENT_PREFIX_PATH', 'CMAKE_PREFIX_PATH', 'COLCON_PREFIX_PATH'):
            sdk_prefixes.update(Path(x) for x in env.get(key, '').split(':') if x)
    sdk = []
    for prefix in sorted(sdk_prefixes):
        for package in (
            prefix / 'share/px4_msgs/package.xml',
            prefix / 'px4_msgs/share/px4_msgs/package.xml',
        ):
            if not package.is_file():
                continue
            row = fingerprint(package)
            try:
                row['version'] = ET.parse(package).findtext('version')
            except (OSError, ET.ParseError) as error:
                row['version_error'] = str(error)
            sdk.append(row)
    serial = command(['fuser', '-v', device])
    return dict(
        schema_version=1,
        collected_utc=datetime.now(timezone.utc).isoformat(),
        host=dict(machine=platform.machine(), hostname=platform.node(), kernel=platform.release()),
        service=service_info,
        entry_root=str(entry),
        entry_root_basis=(
            'service WorkingDirectory'
            if properties.get('WorkingDirectory')
            else 'collector directory fallback; active deployment unknown'
        ),
        project_root=str(project) if project else None,
        processes=process_info,
        files=[fingerprint(path) for path in sorted(paths)],
        source_hash_limit=1000,
        source_hash_limit_reached=source_count >= 1000,
        sdk=dict(
            status='observed' if sdk else 'unknown',
            package_xml=sdk,
            note='Package metadata does not prove wire compatibility or loaded SDK identity.',
        ),
        collector_environment=environments[1],
        serial=dict(device=device, **serial),
        ros=dict(status='not_requested'),
        limitations=[
            'No FC01 shell/parameters, UART bytes or remote device state were read.',
            'On-disk configs and source hashes do not prove current binary used them.',
            'No service/Agent startup, serial open/configuration, or flight publisher.',
        ],
    )


# 固定空间累计有限值的计数/和/范围；无数据用None，不伪造零延迟。
class Aggregate:
    def __init__(self):
        self.count = 0
        self.total = 0.0
        self.minimum = None
        self.maximum = None

    # 仅有限观测进入统计；NaN/Inf不污染均值与边界。
    def add(self, value):
        if not math.isfinite(value):
            return
        self.count += 1
        self.total += value
        self.minimum = value if self.minimum is None else min(self.minimum, value)
        self.maximum = value if self.maximum is None else max(self.maximum, value)

    # 返回当前统计的快照字典，不清除累计水位。
    def report(self):
        return dict(
            count=self.count,
            min=self.minimum,
            max=self.maximum,
            mean=self.total / self.count if self.count else None,
        )


# 每话题分别跟踪接收间隔、源/样本年龄、CDR大小和重复/回退，不能以新包络掩盖旧样本。
class TopicStats:
    def __init__(self):
        self.count = 0
        self.cdr_bytes = 0
        self.last = None
        self.stamps = {}
        self.repeats = dict(message=0, sample=0)
        self.regressions = dict(message=0, sample=0)
        self.gaps = Aggregate()
        self.message_age = Aggregate()
        self.sample_age = Aggregate()
        self.sizes = Aggregate()

    # monotonic用于本机间隔，wall减源微秒用于声明年龄；两类时钟不能互换。
    def observe(self, mono, wall, stamp, sample, size):
        self.count += 1
        self.cdr_bytes += size
        self.sizes.add(size)
        if self.last is not None:
            self.gaps.add(mono - self.last)
        self.last = mono
        for name, value, age in (
            ('message', stamp, self.message_age),
            ('sample', sample, self.sample_age),
        ):
            if type(value) is int and value > 0:
                age.add(wall - value / 1e6)
                old = self.stamps.get(name)
                if old is not None:
                    self.repeats[name] += value == old
                    self.regressions[name] += value < old
                self.stamps[name] = value

    # 频率与CDR速率按实际采样时长计算，不把序列化字节当作串口实际字节。
    def report(self, duration):
        return dict(
            messages=self.count,
            receive_hz=self.count / duration,
            receive_gap_s=self.gaps.report(),
            message_age_s=self.message_age.report(),
            sample_age_s=self.sample_age.report(),
            serialized_cdr_bytes=self.sizes.report(),
            cdr_bytes_per_s=self.cdr_bytes / duration,
            message_stamp_repeats=self.repeats['message'],
            sample_stamp_repeats=self.repeats['sample'],
            message_stamp_regressions=self.regressions['message'],
            sample_stamp_regressions=self.regressions['sample'],
        )


# 读取图端点类型/QoS/数量；发现失败保留unknown，不靠图信息授予控制权。
def endpoint_info(node, topic):
    def convert(endpoint):
        qos = endpoint.qos_profile
        return dict(
            node_name=endpoint.node_name,
            node_namespace=endpoint.node_namespace,
            topic_type=endpoint.topic_type,
            qos={
                key: str(getattr(qos, key, 'unknown'))
                for key in ('reliability', 'durability', 'history', 'depth')
            },
        )

    try:
        publishers = [convert(x) for x in node.get_publishers_info_by_topic(topic)]
        subscribers = [convert(x) for x in node.get_subscriptions_info_by_topic(topic)]
        return dict(
            status='ok',
            publisher_count=len(publishers),
            subscriber_count=len(subscribers),
            publishers=publishers,
            subscribers=subscribers,
        )
    except Exception as error:
        return dict(status='unknown', reason=str(error))


# 延迟导入ROS，创建唯一只读观察节点；finally销毁并关闭该上下文，短窗不证明长期稳定。
def sample_ros(seconds):
    # Deliberately lazy: --help and deployment collection work without ROS installed.
    import rclpy
    import px4_msgs.msg as messages
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.serialization import serialize_message

    result = dict(
        status='ok',
        environment={k: os.environ[k] for k in ENV_KEYS if k in os.environ},
        loaded_px4_msgs=str(messages.__file__),
        topics={},
        notes=[
            'CDR bytes are ROS serialization sizes, not UART throughput.',
            'Age = host wall time minus source timestamp; clock synchronization is unverified.',
            'Observer adds one subscription per sampled topic; counts include it.',
            'Sampling/discovery adds CPU/DDS load; short windows cannot prove long-term stability.',
        ],
    )
    node = None
    subscriptions = []
    stats = {}
    rclpy.init(args=[])
    try:
        node = rclpy.create_node('fc01_passive_collect_' + str(os.getpid()))
        for topic, type_name in TOPICS.items():
            stats[topic] = TopicStats()
            try:
                # 默认参数冻结话题名，避免循环回调串线；记录每条消息真实序列化大小及两个时间戳。
                def receive(msg, name=topic):
                    stats[name].observe(
                        time.monotonic(),
                        time.time(),
                        getattr(msg, 'timestamp', None),
                        getattr(msg, 'timestamp_sample', None),
                        len(serialize_message(msg)),
                    )

                subscriptions.append(
                    node.create_subscription(
                        getattr(messages, type_name), topic, receive, qos_profile_sensor_data
                    )
                )
            except Exception as error:
                result['topics'][topic] = dict(status='unknown', reason=str(error))
        start = time.monotonic()
        deadline = start + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=min(0.1, max(0.0, deadline - time.monotonic())))
        duration = max(time.monotonic() - start, 0.000001)
        result['duration_s'] = duration
        result['nodes'] = [
            dict(name=name, namespace=namespace)
            for name, namespace in node.get_node_names_and_namespaces()
        ]
        result['node_count'] = len(result['nodes'])
        for topic in TOPICS:
            if topic not in result['topics']:
                result['topics'][topic] = dict(
                    status='observed' if stats[topic].count else 'no_messages',
                    **stats[topic].report(duration),
                    endpoints=endpoint_info(node, topic),
                )
        return result
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


# 优先用运行控制器/proc环境与采样环境比较；缺SDK/RMW不能凭默认值认定加载身份一致。
def compare_ros_environment(report):
    sampled = report['ros'].get('environment', {})
    deployed = report['service']['properties'].get('Environment', {})
    basis = 'service Environment'
    available = bool(deployed) or report['service']['properties'].get('LoadState') == 'loaded'
    for process in report['processes']['items']:
        if any(Path(word).name == 'fc01_system_node' for word in process['argv'][:1]):
            deployed = process['environment']
            basis = f'controller PID {process["pid"]}'
            available = 'environment_error' not in process
            break
    result = {}
    for key in ('ROS_DOMAIN_ID', 'ROS_LOCALHOST_ONLY', 'RMW_IMPLEMENTATION', 'AMENT_PREFIX_PATH'):
        # A missing Domain/localhost setting has a defined default; missing SDK/RMW
        # does not identify the middleware default or the actual loaded package.
        default = '0' if key in ('ROS_DOMAIN_ID', 'ROS_LOCALHOST_ONLY') else None
        expected = deployed.get(key, default) if available else None
        observed = sampled.get(key, default)
        result[key] = dict(
            deployed=expected,
            sampled=observed,
            deployed_basis=basis,
            status=(
                'unknown'
                if expected is None or observed is None
                else 'match' if expected == observed else 'mismatch'
            ),
        )
    return result


# 输出必须新目录，0..30秒窗口显式选择；ROS工作者继承调用方环境且外层再加超时。
def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--output', type=Path, help='New report directory; default ~/FC01_communications_<UTC>'
    )
    parser.add_argument('--service', default='fc01-navigation.service')
    parser.add_argument(
        '--device', default='/dev/ttyS2', help='Observe ownership only; never open/configure UART'
    )
    parser.add_argument(
        '--project-root',
        type=Path,
        help='Explicit SDK/base-config location; service identity still recorded',
    )
    parser.add_argument(
        '--ros-seconds',
        type=float,
        default=0.0,
        help='Optional passive ROS window, 0..30 s; suggested 10',
    )
    parser.add_argument('--_ros-worker', action='store_true', help=argparse.SUPPRESS)
    args = parser.parse_args(argv)
    if not math.isfinite(args.ros_seconds) or not 0 <= args.ros_seconds <= 30:
        parser.error('--ros-seconds must be finite and within 0..30')
    if not args.service or args.service.startswith('-') or '/' in args.service:
        parser.error('--service must be a system unit name')
    if not args.device.startswith('/dev/'):
        parser.error('--device must be an absolute /dev path')
    if args._ros_worker:
        try:
            result = sample_ros(args.ros_seconds)
        except Exception as error:
            result = dict(status='unknown', reason=f'{type(error).__name__}: {error}')
        args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n')
        return 0
    output = args.output or Path.home() / (
        'FC01_communications_' + datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
    )
    os.umask(0o077)
    output.mkdir(parents=True, exist_ok=False)
    report = collect(args.service, args.device, args.project_root)
    if args.ros_seconds:
        # Inherit the caller's sourced SDK, and record that environment explicitly.
        # Never source/execute deployment scripts or impersonate a service environment.
        child = run_command(
            [
                sys.executable,
                str(Path(__file__).resolve()),
                '--_ros-worker',
                '--ros-seconds',
                str(args.ros_seconds),
                '--output',
                str(output / 'ros_topics.json'),
            ],
            timeout=args.ros_seconds + 12,
        )
        try:
            report['ros'] = json.loads((output / 'ros_topics.json').read_text())
        except (OSError, ValueError):
            report['ros'] = dict(
                status='unknown',
                reason=child.get('reason')
                or child.get('stderr')
                or 'ROS child did not produce a report',
            )
        if report['ros'].get('environment') is not None:
            report['ros']['deployment_environment_comparison'] = compare_ros_environment(report)
    (output / 'report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
    print(output / 'report.json')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
