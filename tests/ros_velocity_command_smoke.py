#!/usr/bin/env python3
# 独立速度诊断的真实ROS传输回归：只启动合成VelocityCommand源与只读诊断节点。
# 强制本机发现并拒绝已有/fmu/FC01/Agent；所有样本值和超时均为测试夹具，不给飞行权限。
"""Opt-in, real ROS 2 transport test for the read-only velocity diagnostic.

Source ROS 2 Humble and the newly built jikl_fc01_system install, then run with
an explicitly chosen, unused ROS_DOMAIN_ID other than 0. This script forces
localhost discovery and clears discovery-server/profile overrides. It starts
only a synthetic VelocityCommand publisher and the standalone diagnostic; it
does not start FC01, an Agent, a serial connection, or a flight controller.

The packet values, snapshot periods and timeout used below are test fixtures,
not flight parameters or an agreement about the production command lifetime.
This verifies local ROS transport/diagnostics, not navigation or flight safety.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time


TYPE_NAME = 'jikl_fc01_system/msg/VelocityCommand'
TOPIC = '/fc01/vision/velocity_command'
DIAGNOSTIC_NODE = 'fc01_velocity_command_diagnostic'
TEST_TIMEOUT_SEC = 1.0


# 统一断言入口，失败时抛出含具体边界原因的RuntimeError。
def require(condition, reason):
    if not condition:
        raise RuntimeError(reason)


# 核对真实生成的类型身份、字段类型、默认值和CDR往返，不能用.msg注释代替默认值验证。
def check_generated_interface(message_type, serialize, deserialize):
    """Check the generated type, including defaults which comments do not set."""
    expected = {
        'schema_version': 'uint16',
        'sequence': 'uint64',
        'stamp': 'builtin_interfaces/Time',
        'reset_counter': 'uint32',
        'source_id': 'string',
        'frame_id': 'string',
        'valid': 'boolean',
        'vx': 'double',
        'vy': 'double',
        'vz': 'double',
    }
    require(
        message_type.__module__ == 'jikl_fc01_system.msg._velocity_command',
        'loaded a different ROS message type',
    )
    require(
        message_type.get_fields_and_field_types() == expected,
        'generated VelocityCommand fields differ from the agreed wire contract',
    )
    default = message_type()
    require(
        default.schema_version == 1
        and default.sequence == 0
        and default.reset_counter == 0
        and default.stamp.sec == 0
        and default.stamp.nanosec == 0
        and default.source_id == ''
        and default.frame_id == ''
        and default.valid is False
        and (default.vx, default.vy, default.vz) == (0.0, 0.0, 0.0),
        'generated message defaults differ; frame_id must be filled explicitly',
    )
    fixture = message_type(
        schema_version=1,
        sequence=19,
        reset_counter=3,
        source_id='synthetic-wire',
        frame_id='body_frd',
        valid=True,
        vx=0.17,
        vy=-0.23,
        vz=-0.11,
    )
    fixture.stamp.sec = 123
    fixture.stamp.nanosec = 456
    require(
        deserialize(serialize(fixture), message_type) == fixture,
        'generated ROS serialization round trip changed the message',
    )
    return {
        'type': TYPE_NAME,
        'fields': expected,
        'default_frame_id': default.frame_id,
        'serialization_round_trip': True,
    }


# 在显式非零隔离域内测试best-effort/reliable、异常历史、超时和最终退出落盘。
def main():
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--diagnostic', type=Path, default=root / 'native/tools/velocity_command_diagnostic.py'
    )
    parser.add_argument(
        '--report', type=Path, help='optional JSON result; use a path outside the source tree'
    )
    args = parser.parse_args()
    try:
        domain = int(os.environ.get('ROS_DOMAIN_ID', '0'))
    except ValueError:
        parser.error('ROS_DOMAIN_ID must be an explicitly selected integer')
    if not 1 <= domain <= 232:
        parser.error('select an unused ROS_DOMAIN_ID in 1..232 for this isolated test')
    if not args.diagnostic.is_file():
        parser.error('build the message and provide the standalone diagnostic first')

    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    for variable in (
        'ROS_DISCOVERY_SERVER',
        'CYCLONEDDS_URI',
        'FASTRTPS_DEFAULT_PROFILES_FILE',
        'FASTDDS_DEFAULT_PROFILES_FILE',
        'RMW_FASTRTPS_USE_QOS_FROM_XML',
        'ROS_STATIC_PEERS',
    ):
        os.environ.pop(variable, None)
    os.environ['PYTHONDONTWRITEBYTECODE'] = '1'

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from rclpy.serialization import serialize_message, deserialize_message
    from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
    from rcl_interfaces.srv import DescribeParameters, GetParameters, SetParameters
    from rosidl_runtime_py.utilities import get_message
    from jikl_fc01_system.msg import VelocityCommand

    require(get_message(TYPE_NAME) is VelocityCommand, 'ROS type identity mismatch')
    result = {
        'domain_id': domain,
        'localhost_only': True,
        'control_authority': False,
        'interface': check_generated_interface(
            VelocityCommand, serialize_message, deserialize_message
        ),
        'checks': [],
        'test_timeout_sec': TEST_TIMEOUT_SEC,
        'timeout_is_test_fixture': True,
    }
    rclpy.init(args=[])
    node = Node('fc01_velocity_command_smoke_source')
    publishers = []
    process = None
    logs = None
    snapshot_path = None

    # 每次等待都扫描图，若出现飞控端点、FC01或Agent即拒绝继续合成流量。
    def isolated_graph():
        topics = node.get_topic_names_and_types()
        require(
            not any(name.startswith('/fmu/') for name, _ in topics),
            'isolated test domain contains /fmu endpoints; choose another domain',
        )
        require(
            not any(
                name == 'fc01_system' or 'micro_xrce' in name.lower()
                for name in node.get_node_names()
            ),
            'isolated test domain contains FC01 or an Agent node',
        )

    # 读取独立诊断JSON；文件尚未出现或原子替换中的短暂解析失败只表示尚无快照。
    def snapshot():
        try:
            return json.loads(snapshot_path.read_text(encoding='utf-8'))
        except (FileNotFoundError, json.JSONDecodeError):
            return None

    # spin并不断核对隔离图和诊断进程；只有满足谓词且control_authority严格false才返回。
    def wait_for(predicate, reason, timeout=12.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if process is not None:
                require(process.poll() is None, 'standalone diagnostic exited during ' + reason)
            rclpy.spin_once(node, timeout_sec=0.05)
            isolated_graph()
            current = snapshot()
            if current is not None and predicate(current):
                require(
                    current.get('control_authority') is False,
                    'diagnostic unexpectedly claims control authority',
                )
                return current
        raise RuntimeError('timed out waiting for ' + reason)

    # 读取缺省为0的统计计数，便于检测某次异常是否实际新增。
    def count(current, key):
        return current.get('counters', {}).get(key, 0)

    # 真正调用ROS参数服务，等待期间仍核对域隔离；finally销毁服务client。
    def parameter_call(service_type, suffix, request):
        client = node.create_client(service_type, '/' + DIAGNOSTIC_NODE + '/' + suffix)
        try:
            require(
                client.wait_for_service(timeout_sec=5.0),
                'diagnostic parameter service missing: ' + suffix,
            )
            future = client.call_async(request)
            deadline = time.monotonic() + 5.0
            while not future.done() and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
                isolated_graph()
            require(future.done(), 'diagnostic parameter call timed out: ' + suffix)
            return future.result()
        finally:
            node.destroy_client(client)

    # 核对描述符只读、运行期修改被拒绝，且可见参数仍为启动值。
    def check_startup_parameters_are_read_only():
        names = [
            'topic',
            'qos_depth',
            'reliable',
            'snapshot_period_sec',
            'timeout_sec',
            'max_source_age_sec',
            'future_tolerance_sec',
            'max_speed_mps',
            'expected_source_id',
            'output_dir',
            'log_max_bytes',
            'log_backup_count',
            'log_min_interval_sec',
        ]
        described = parameter_call(
            DescribeParameters, 'describe_parameters', DescribeParameters.Request(names=names)
        )
        require(
            len(described.descriptors) == len(names)
            and all(value.read_only for value in described.descriptors),
            'startup-only diagnostic parameter advertised as mutable',
        )
        changes = [
            Parameter(
                name='timeout_sec',
                value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=2.0),
            ),
            Parameter(
                name='expected_source_id',
                value=ParameterValue(
                    type=ParameterType.PARAMETER_STRING, string_value='unbound-change'
                ),
            ),
        ]
        changed = parameter_call(
            SetParameters, 'set_parameters', SetParameters.Request(parameters=changes)
        )
        require(
            len(changed.results) == 2 and all(not value.successful for value in changed.results),
            'startup-only parameter update silently succeeded',
        )
        current = parameter_call(
            GetParameters,
            'get_parameters',
            GetParameters.Request(names=['timeout_sec', 'expected_source_id']),
        )
        require(
            len(current.values) == 2
            and current.values[0].double_value == TEST_TIMEOUT_SEC
            and current.values[1].string_value == '',
            'rejected runtime parameter update changed the startup configuration',
        )

    # 构造显式body_frd目标速度及当前源时戳，序号/复位/来源/有效性按场景变化。
    def make_message(sequence, reset=0, source='synthetic-smoke-A', valid=True):
        message = VelocityCommand(
            sequence=sequence,
            reset_counter=reset,
            source_id=source,
            frame_id='body_frd',
            valid=valid,
            vx=0.17,
            vy=-0.23,
            vz=-0.11,
        )
        message.stamp = node.get_clock().now().to_msg()
        return message

    # 固定volatile与深度5，只改变可靠性，用真实图匹配验证传输契约。
    def qos(reliable):
        return QoSProfile(
            depth=5,
            reliability=(ReliabilityPolicy.RELIABLE if reliable else ReliabilityPolicy.BEST_EFFORT),
            durability=DurabilityPolicy.VOLATILE,
        )

    # 登记新发布者，便于冲突计数验证和finally统一回收。
    def new_publisher(reliable):
        publisher = node.create_publisher(VelocityCommand, TOPIC, qos(reliable))
        publishers.append(publisher)
        return publisher

    # 销毁ROS发布者并移除登记，保证随后源计数观察对应真实图。
    def destroy_publisher(publisher):
        node.destroy_publisher(publisher)
        publishers.remove(publisher)

    # 等唯一兼容订阅发现后只发送一次；重复发送会伪造重复序号并掩盖测试失效。
    def publish_and_wait(publisher, message, predicate, reason):
        # Publish once after endpoint discovery. Repeating a fixture here would
        # manufacture sequence duplicates and hide a failed duplicate check.
        wait_for(
            lambda _: publisher.get_subscription_count() == 1,
            'one compatible diagnostic subscription',
        )
        publisher.publish(message)
        return wait_for(predicate, reason)

    # 优先SIGINT并有限等待；返回真实退出码，清空句柄避免重复停止。
    def stop_process():
        nonlocal process
        if process is None:
            return None
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.terminate()
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=5)
        code = process.returncode
        process = None
        return code

    try:
        with tempfile.TemporaryDirectory(prefix='fc01_velocity_ros_smoke_') as temporary:
            directory = Path(temporary)
            logs = tempfile.TemporaryFile(mode='w+')

            # 每次使用独立临时输出目录与显式测试参数，等待首份快照再进入场景。
            def start_diagnostic(reliable, *, name=None, overrides=None):
                nonlocal process, snapshot_path
                output = directory / (name or ('reliable' if reliable else 'best_effort'))
                output.mkdir()
                snapshot_path = output / 'velocity_command_diagnostic.json'
                command = [sys.executable, str(args.diagnostic), '--ros-args']
                parameters = {
                    'topic': TOPIC,
                    'output_dir': str(output),
                    'reliable': str(reliable).lower(),
                    'timeout_sec': str(TEST_TIMEOUT_SEC),
                    'snapshot_period_sec': '0.05',
                    'log_min_interval_sec': '0.01',
                }
                parameters.update(overrides or {})
                for key, value in parameters.items():
                    command += ['-p', key + ':=' + value]
                process = subprocess.Popen(
                    command, stdin=subprocess.DEVNULL, stdout=logs, stderr=subprocess.STDOUT
                )
                return wait_for(lambda _: True, 'initial diagnostic snapshot')

            # Allow graph discovery before publishing any fixture.
            initial_deadline = time.monotonic() + 1.0
            while time.monotonic() < initial_deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
                isolated_graph()

            start_diagnostic(False)
            check_startup_parameters_are_read_only()
            unchanged = wait_for(
                lambda current: current.get('limits', {}).get('timeout_sec') == TEST_TIMEOUT_SEC,
                'startup override retained after rejected parameter update',
            )
            require(
                unchanged.get('expected_source_id') is None,
                'rejected source binding appeared in receiver configuration',
            )
            result['checks'].append('startup_overrides_and_read_only_runtime_parameters')
            best_effort = new_publisher(False)
            wait_for(lambda current: current.get('publisher_count') == 1, 'one best effort source')
            subscriptions = node.get_subscriptions_info_by_topic(TOPIC)
            require(
                len(subscriptions) == 1
                and subscriptions[0].topic_type == TYPE_NAME
                and subscriptions[0].qos_profile.reliability == ReliabilityPolicy.BEST_EFFORT,
                'diagnostic default subscription has wrong type or reliability',
            )
            # 首个best-effort包需保持全部载荷和测量时戳；只配接收超时不应形成可用控制候选。
            first = make_message(10)
            current = publish_and_wait(
                best_effort,
                first,
                lambda current: (current.get('last_message') or {}).get('sequence') == 10,
                'best effort packet delivery',
            )
            last = current['last_message']
            for key in (
                'schema_version',
                'sequence',
                'reset_counter',
                'source_id',
                'frame_id',
                'valid',
                'vx',
                'vy',
                'vz',
            ):
                require(last.get(key) == getattr(first, key), 'wire payload mismatch: ' + key)
            require(
                last.get('stamp_ns') == first.stamp.sec * 1000000000 + first.stamp.nanosec,
                'source measurement timestamp was not preserved',
            )
            require(
                current.get('cache_usable') is False and current.get('current_candidate') is None,
                'test receive timeout alone must not qualify a command candidate',
            )
            result['checks'].append('best_effort_type_and_payload')
            result['checks'].append('unconfigured_policy_does_not_qualify_cache')

            # 依次重发同序号、发倒序包，计数须各自增加；拒绝数据不得伪造新鲜有效样本。
            before = count(current, 'duplicate')
            current = publish_and_wait(
                best_effort,
                first,
                lambda current: count(current, 'duplicate') > before,
                'duplicate observation',
            )
            before = count(current, 'out_of_order')
            current = publish_and_wait(
                best_effort,
                make_message(9),
                lambda current: count(current, 'out_of_order') > before,
                'out of order observation',
            )
            result['checks'].append('duplicate_and_out_of_order')

            before = count(current, 'invalid')
            current = publish_and_wait(
                best_effort,
                make_message(11, valid=False),
                lambda current: count(current, 'invalid') > before
                and current.get('last_message', {}).get('valid') is False,
                'explicit invalid observation',
            )
            # 后续有效包可以恢复最新观察，但先前invalid异常历史须保留。
            remembered_anomaly = current.get('last_anomaly')
            require(remembered_anomaly is not None, 'invalid message did not retain an anomaly')
            current = publish_and_wait(
                best_effort,
                make_message(12),
                lambda current: current.get('last_message', {}).get('sequence') == 12,
                'fresh valid packet after invalid observation',
            )
            require(
                current.get('last_anomaly') is not None, 'valid tail erased all anomaly history'
            )
            result['checks'].append('invalid_and_retained_anomaly')

            # 复位与来源变化是两个独立流事件；它们不会自动产生飞行权限。
            before = count(current, 'reset_change')
            current = publish_and_wait(
                best_effort,
                make_message(1, reset=1),
                lambda current: count(current, 'reset_change') > before
                and current.get('last_message', {}).get('reset_counter') == 1,
                'producer reset with sequence restart',
            )
            before = count(current, 'source_change')
            current = publish_and_wait(
                best_effort,
                make_message(1, reset=1, source='synthetic-smoke-B'),
                lambda current: count(current, 'source_change') > before
                and current.get('last_message', {}).get('source_id') == 'synthetic-smoke-B',
                'source identity change',
            )
            result['checks'].append('reset_and_source_changes')

            # 两发布者冲突必须被记录；恢复唯一发布者后才测试reliable到best-effort传输。
            before = count(current, 'source_count')
            reliable = new_publisher(True)
            wait_for(
                lambda current: current.get('publisher_count') == 2
                and count(current, 'source_count') > before,
                'two source conflict observation',
            )
            destroy_publisher(best_effort)
            wait_for(lambda current: current.get('publisher_count') == 1, 'source count recovery')
            current = publish_and_wait(
                reliable,
                make_message(2, reset=1, source='synthetic-smoke-B'),
                lambda current: current.get('last_message', {}).get('sequence') == 2,
                'reliable publisher delivery to best effort diagnostic',
            )
            require(
                count(current, 'source_count') >= 1, 'source count transitions were not recorded'
            )
            result['checks'].append('source_count_and_reliable_to_best_effort')

            # 静默直到显式测试超时：原last_message仍应保持序号2，不制造新的线缆样本。
            before = count(current, 'timeout')
            current = wait_for(
                lambda current: count(current, 'timeout') > before
                and current.get('freshness') == 'EXPIRED',
                'explicit test timeout after packet silence',
            )
            require(
                current['last_message']['sequence'] == 2, 'timeout manufactured a new wire sample'
            )
            result['checks'].append('explicit_test_timeout')

            endpoints = node.get_publisher_names_and_types_by_node(DIAGNOSTIC_NODE, '/')
            require(
                not any(name.startswith('/fmu/in/') for name, _ in endpoints),
                'diagnostic exposes a hardware publisher',
            )
            result['diagnostic_publishers'] = endpoints
            result['checks'].append('diagnostic_has_no_fmu_publishers')
            destroy_publisher(reliable)
            wait_for(lambda current: current.get('publisher_count') == 0, 'all sources removed')
            stop_process()
            wait_deadline = time.monotonic() + 8.0
            while DIAGNOSTIC_NODE in node.get_node_names() and time.monotonic() < wait_deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
            require(
                DIAGNOSTIC_NODE not in node.get_node_names(),
                'first diagnostic still appears in graph before reliability test',
            )

            start_diagnostic(True)
            reliable = new_publisher(True)
            wait_for(lambda current: current.get('publisher_count') == 1, 'one reliable source')
            publish_and_wait(
                reliable,
                make_message(1),
                lambda current: (current.get('last_message') or {}).get('sequence') == 1,
                'reliable subscription delivery',
            )
            subscriptions = node.get_subscriptions_info_by_topic(TOPIC)
            require(
                len(subscriptions) == 1
                and subscriptions[0].qos_profile.reliability == ReliabilityPolicy.RELIABLE,
                'explicit reliable subscription was not applied',
            )
            result['checks'].append('reliable_to_reliable')
            isolated_graph()
            stop_process()
            destroy_publisher(reliable)
            wait_deadline = time.monotonic() + 8.0
            while DIAGNOSTIC_NODE in node.get_node_names() and time.monotonic() < wait_deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
            require(
                DIAGNOSTIC_NODE not in node.get_node_names(),
                'reliable diagnostic still appears before shutdown test',
            )

            # 延长快照/日志节流期，让invalid事件在SIGINT前尚未刷盘，验证退出强制保存不遗漏。
            # Deliberately long fixture periods keep the invalid event pending
            # until SIGINT; this checks finalization after rclpy shuts its context.
            start_diagnostic(
                True,
                name='shutdown',
                overrides={'snapshot_period_sec': '60.0', 'log_min_interval_sec': '60.0'},
            )
            reliable = new_publisher(True)
            wait_for(
                lambda _: reliable.get_subscription_count() == 1,
                'shutdown test reliable subscription',
            )
            reliable.publish(make_message(99, valid=False))
            delivery_deadline = time.monotonic() + 1.0
            while time.monotonic() < delivery_deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
                isolated_graph()
            exit_code = stop_process()
            require(exit_code == 0, 'SIGINT diagnostic finalization failed')
            final = snapshot()
            require(
                final is not None
                and final.get('receiver_running') is False
                and (final.get('last_message') or {}).get('sequence') == 99
                and 'COMMAND_INVALID' in final.get('latest_errors', []),
                'shutdown lost the final invalid observation or running-state marker',
            )
            records = [
                json.loads(line)
                for line in snapshot_path.with_name('velocity_command_diagnostic.log')
                .read_text()
                .splitlines()
            ]
            require(
                any(
                    'COMMAND_INVALID'
                    in (entry.get('reasons', []) + entry.get('coalesced_reasons', []))
                    for entry in records
                ),
                'SIGINT lost the throttled pending anomaly from the log',
            )
            require(
                final.get('control_authority') is False and not final.get('cache_usable'),
                'stopped diagnostic retained a usable control candidate',
            )
            result['checks'].append('sigint_final_snapshot_and_forced_anomaly_flush')
            result['passed'] = True
            if args.report:
                args.report.write_text(
                    json.dumps(result, ensure_ascii=False, indent=2) + '\n', encoding='utf-8'
                )
            print('REAL_ROS_VELOCITY_COMMAND_PASS ' + json.dumps(result, ensure_ascii=False))
    except Exception:
        if logs is not None:
            logs.seek(0)
            print(logs.read()[-8000:], file=sys.stderr)
        raise
    # 正常或异常退出都停止诊断、销毁全部发布者和节点，并关闭日志文件。
    finally:
        stop_process()
        for publisher in list(publishers):
            destroy_publisher(publisher)
        node.destroy_node()
        rclpy.shutdown()
        if logs is not None:
            logs.close()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
