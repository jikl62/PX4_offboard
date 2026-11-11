#!/usr/bin/env python3
# 真实ROS视觉适配冒烟：只在明确隔离的本机域生成/fc01_demo数据，不启动控制器。
# 验证转换、来源冲突撤销、启动参数只读和SIGINT清理；不是飞行验收。
"""Opt-in real ROS adapter check. Creates only /fc01_demo/** data endpoints."""
from pathlib import Path
import argparse
import json
import math
import os
import signal
import subprocess
import sys
import tempfile
import time


# 启动独立适配器，发布合成ENU/FLU状态并从真实话题接收VisionState。
def main():
    parser = argparse.ArgumentParser(description='只在隔离 ROS 域验证视觉适配器，不启动控制器。')
    parser.add_argument('--adapter', type=Path)
    parser.add_argument('--report', type=Path)
    parser.add_argument('--check-resource-budget', action='store_true')
    options = parser.parse_args()
    # 合成来源不能与真实控制服务混用 ROS 域；要求操作者明确选用本机测试域。
    domain = int(os.environ.get('ROS_DOMAIN_ID', '0'))
    if os.environ.get('ROS_LOCALHOST_ONLY') != '1' or not 0 < domain < 233:
        raise RuntimeError('set ROS_LOCALHOST_ONLY=1 and a nonzero isolated ROS_DOMAIN_ID')
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from nav_msgs.msg import Odometry
    from jikl_fc01_system.msg import VisionHealth, VisionState
    from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
    from rcl_interfaces.srv import DescribeParameters, GetParameters, SetParameters

    root = Path(__file__).resolve().parents[1]
    params = dict(
        input_topic='/fc01_demo/odom',
        health_topic='/fc01_demo/health',
        output_topic='/fc01_demo/state',
        source_id='synthetic-ros-smoke',
        input_frame_id='demo_enu',
        input_body_frame_id='demo_flu',
        world_convention='ENU',
        body_convention='FLU',
        extrinsics_id='synthetic-identity',
        alignment_id='synthetic-identity',
        body_from_sensor_wxyz='[1.0,0.0,0.0,0.0]',
        sensor_position_body_frd='[0.0,0.0,0.0]',
        world_alignment_wxyz='[1.0,0.0,0.0,0.0]',
        world_translation_ned='[0.0,0.0,0.0]',
    )
    adapter_path = options.adapter or root / 'native/tools/vision_odometry_adapter.py'
    argv = [sys.executable, str(adapter_path), '--ros-args']
    for name, value in params.items():
        argv += ['-p', name + ':=' + value]
    with tempfile.TemporaryFile(mode='w+') as logs:
        process = subprocess.Popen(
            argv, stdin=subprocess.DEVNULL, stdout=logs, stderr=subprocess.STDOUT
        )
        node = None
        rclpy.init()
        try:
            node = Node('fc01_vision_smoke_source')
            sensor = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
            reliable = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
            odom_pub = node.create_publisher(Odometry, params['input_topic'], sensor)
            health_pub = node.create_publisher(VisionHealth, params['health_topic'], reliable)
            received = []
            subscription = node.create_subscription(
                VisionState, params['output_topic'], received.append, sensor
            )
            sequence = 0
            source_override = None

            # 经真实参数服务调用并有限等待，finally销毁client，不仅检查源码声明。
            def parameter_call(service_type, suffix, request):
                # 参数必须经真实 ROS 服务验证，不能只检查源码中的只读声明。
                client = node.create_client(service_type, '/fc01_vision_odometry_adapter/' + suffix)
                try:
                    if not client.wait_for_service(timeout_sec=5.0):
                        raise RuntimeError('adapter parameter service unavailable: ' + suffix)
                    future = client.call_async(request)
                    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
                    if not future.done():
                        raise RuntimeError('adapter parameter service timed out: ' + suffix)
                    return future.result()
                finally:
                    node.destroy_client(client)

            names = list(params) + [
                'qos_depth',
                'output_reliable',
                'odometry_reliable',
                'health_reliable',
                'pair_timeout',
                'max_pending',
                'use_sim_time',
            ]
            descriptors = parameter_call(
                DescribeParameters, 'describe_parameters', DescribeParameters.Request(names=names)
            ).descriptors
            if len(descriptors) != len(names) or not all(item.read_only for item in descriptors):
                raise RuntimeError('startup adapter parameters must advertise read_only')
            # 检查来源、外参和时钟域：运行期拒绝修改，不能显示成功但继续使用旧配置。
            changes = [
                Parameter(
                    name='source_id',
                    value=ParameterValue(
                        type=ParameterType.PARAMETER_STRING, string_value='rejected-runtime-source'
                    ),
                ),
                Parameter(
                    name='world_translation_ned',
                    value=ParameterValue(
                        type=ParameterType.PARAMETER_DOUBLE_ARRAY,
                        double_array_value=[100.0, 0.0, 0.0],
                    ),
                ),
                Parameter(
                    name='use_sim_time',
                    value=ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=True),
                ),
            ]
            updates = parameter_call(
                SetParameters, 'set_parameters', SetParameters.Request(parameters=changes)
            ).results
            if len(updates) != 3 or any(item.successful for item in updates):
                raise RuntimeError('startup-only adapter parameter update silently succeeded')
            visible = parameter_call(
                GetParameters,
                'get_parameters',
                GetParameters.Request(names=[item.name for item in changes]),
            ).values
            if (
                visible[0].string_value != params['source_id']
                or list(visible[1].double_array_value) != [0.0, 0.0, 0.0]
                or visible[2].bool_value
            ):
                raise RuntimeError(
                    'rejected adapter parameter update changed visible configuration'
                )

            # Odometry与Health使用同一时间戳；给出固定位置、速度、角速度和显式合成健康标记。
            def publish():
                nonlocal sequence
                sequence += 1
                stamp = node.get_clock().now().to_msg()
                odom = Odometry()
                odom.header.stamp = stamp
                odom.header.frame_id = 'demo_enu'
                odom.child_frame_id = 'demo_flu'
                odom.pose.pose.position.x = 1.0
                odom.pose.pose.position.y = 2.0
                odom.pose.pose.position.z = 3.0
                odom.pose.pose.orientation.w = 1.0
                odom.twist.twist.linear.x = 0.4
                odom.twist.twist.linear.y = 0.5
                odom.twist.twist.linear.z = 0.6
                odom.twist.twist.angular.x = 0.1
                odom.twist.twist.angular.y = 0.2
                odom.twist.twist.angular.z = 0.3
                health = VisionHealth()
                health.stamp = stamp
                health.sequence = sequence
                health.source_id = (
                    source_override if source_override is not None else params['source_id']
                )
                health.frame_id = 'demo_enu'
                health.body_frame_id = 'demo_flu'
                for flag in (
                    'reset_valid',
                    'initialized',
                    'tracking',
                    'calibration_valid',
                    'frame_alignment_valid',
                    'time_sync_valid',
                    'extrinsics_valid',
                    'metric_scale_valid',
                    'position_valid',
                    'velocity_valid',
                    'attitude_valid',
                    'angular_velocity_valid',
                    'quality_valid',
                ):
                    setattr(health, flag, True)
                health.angular_velocity_source = 'synthetic-imu'
                health.quality = 1.0
                health.extrinsics_id = health.alignment_id = 'synthetic-identity'
                health_pub.publish(health)
                odom_pub.publish(odom)

            # 持续送一对新样本并spin，等断言成立；适配器提前退出或超时立即失败。
            def wait_for(predicate):
                deadline = time.monotonic() + 8
                while time.monotonic() < deadline:
                    if process.poll() is not None:
                        raise RuntimeError('adapter exited before data check')
                    publish()
                    rclpy.spin_once(node, timeout_sec=0.05)
                    if predicate():
                        return
                raise RuntimeError('ROS adapter data check timed out')

            wait_for(lambda: any((x.valid_fields & 71) == 71 for x in received))
            valid = next(x for x in reversed(received) if x.valid_fields & 71 == 71)
            for actual, want in zip(valid.position, (2.0, 1.0, -3.0)):
                if abs(actual - want) > 1e-8:
                    raise RuntimeError('ENU position conversion mismatch')
            for actual, want in zip(valid.velocity, (0.5, 0.4, -0.6)):
                if abs(actual - want) > 1e-8:
                    raise RuntimeError('child-frame velocity conversion mismatch')
            if abs(sum(x * x for x in valid.attitude) - 1) > 1e-8:
                raise RuntimeError('invalid output quaternion')
            if valid.valid_fields & 48 or not all(math.isnan(x) for x in valid.pose_covariance):
                raise RuntimeError('untransformed covariance was marked valid')
            if options.check_resource_budget:
                # 通过真实 DDS 送入超出 256 UTF-8 字节的身份；不制造巨型网络流量或控制端点。
                # 两种编码都必须撤销；恢复后只能用新的完整配对重新发布有效测量。
                for oversized_source in ('x' * 257, 'é' * 129):
                    marker = b'state_quality=VISION_IDENTITY_RESOURCE_LIMIT'
                    # pread 不移动子进程共享的日志文件偏移；为每类输入单独观察新的拒绝记录。
                    before_rejections = os.pread(logs.fileno(), 1024 * 1024, 0).count(marker)
                    source_override = oversized_source
                    last_sequence = max(x.sequence for x in received)
                    wait_for(
                        lambda: any(
                            x.sequence > last_sequence and x.valid_fields == 0 for x in received
                        )
                        and os.pread(logs.fileno(), 1024 * 1024, 0).count(marker)
                        > before_rejections
                    )
                    invalid = max(received, key=lambda x: x.sequence)
                    if invalid.source_id != params['source_id']:
                        raise RuntimeError('resource rejection copied untrusted identity')
                    # 每类坏身份之间先恢复完整配对，保证前一轮在途撤销不能满足下一轮断言。
                    source_override = None
                    last_sequence = max(x.sequence for x in received)
                    wait_for(
                        lambda: any(
                            x.sequence > last_sequence and x.valid_fields & 71 == 71
                            for x in received
                        )
                    )
                valid = next(x for x in reversed(received) if x.valid_fields & 71 == 71)
            # 新增第二个原始Odometry发布者，要求后续VisionState的valid_fields撤销到0。
            last_sequence = valid.sequence
            duplicate = node.create_publisher(Odometry, params['input_topic'], sensor)
            wait_for(
                lambda: any(x.sequence > last_sequence and x.valid_fields == 0 for x in received)
            )
            node.destroy_publisher(duplicate)
            # 从真实图查询适配器发布端点，不得出现/fmu/in硬件控制发布者。
            endpoints = node.get_publisher_names_and_types_by_node(
                'fc01_vision_odometry_adapter', '/'
            )
            if any(name.startswith('/fmu/in/') for name, _ in endpoints):
                raise RuntimeError('unexpected hardware endpoint')
            if subscription is None:
                raise RuntimeError('subscription was not retained')
            # 正常 SIGINT 必须以零状态退出，不能被重复关闭上下文的异常覆盖。
            process.send_signal(signal.SIGINT)
            process.wait(timeout=5.0)
            logs.seek(0)
            final_log = logs.read()
            if process.returncode != 0 or 'Traceback (most recent call last)' in final_log:
                raise RuntimeError('adapter failed clean SIGINT shutdown: ' + final_log[-2000:])
            if options.check_resource_budget and 'VISION_IDENTITY_RESOURCE_LIMIT' not in final_log:
                raise RuntimeError('resource budget rejection was not observed in adapter log')
            report = dict(
                passed=True,
                domain_id=domain,
                localhost_only=True,
                adapter_path=str(adapter_path),
                checks=[
                    'read_only_startup_parameters',
                    'valid_conversion',
                    'raw_conflict_revoked',
                    'no_fmu_publishers',
                    'sigint_clean_exit',
                ],
                publisher_endpoints=endpoints,
                shutdown_exit_code=process.returncode,
            )
            if options.check_resource_budget:
                report['checks'] += [
                    'utf8_identity_budget_revoked',
                    'fresh_pair_after_resource_rejection',
                ]
            if options.report is not None:
                options.report.write_text(json.dumps(report, indent=2) + '\n')
            print(
                'REAL_ROS_VISION_ADAPTER_PASS valid_conversion=1 raw_conflict_revoked=1 fmu_publishers=0 read_only_parameters=1 sigint_clean_exit=1'
            )
        except Exception:
            logs.seek(0)
            print(logs.read()[-5000:], file=sys.stderr)
            raise
        # 失败也销毁ROS节点并结束适配器，避免测试残留影响下轮发现数量。
        finally:
            if node is not None:
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.terminate()
                    process.wait(timeout=5)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
