#!/usr/bin/env python3
# 视觉接口主机基准：计量ROS CDR序列化、独立速度诊断、Odometry转换及状态验证开销。
# 不创建ROS端点；结果不包含DDS调度/串口开销，也不能当作香橙派实时性能或剩余带宽证据。
"""Measure host serialization/adapter costs without creating any ROS endpoints.

Source Humble, pinned px4_msgs and the current jikl_fc01_system install first.
Rates below describe explicit traffic scenarios, not validated sensor settings.
CDR sizes exclude RTPS, XRCE, serial framing and other traffic on the UART.
"""
import argparse
import importlib.util
import json
import platform
from pathlib import Path
import resource
import sys
import time


ROOT = Path(__file__).resolve().parents[1]


# 按绝对源码路径载入native/tools纯逻辑，防止测到其他安装版本。
def load_tool(name):
    path = ROOT / 'native/tools' / (name + '.py')
    spec = importlib.util.spec_from_file_location('fc01_benchmark_' + name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


# 先预热20次，再分别记录进程CPU时间和墙钟分位数；分位数含主机计时开销。
def measure(operation, samples):
    for index in range(20):
        operation(index)
    cpu_start = time.process_time()
    started = time.perf_counter()
    durations = []
    for index in range(samples):
        before = time.perf_counter()
        operation(index + 20)
        durations.append((time.perf_counter() - before) * 1e6)
    elapsed = time.perf_counter() - started
    cpu = time.process_time() - cpu_start
    durations.sort()
    return dict(samples=samples, wall_seconds=elapsed, cpu_seconds=cpu,
                mean_wall_us=elapsed / samples * 1e6,
                mean_cpu_us=cpu / samples * 1e6,
                p50_us=durations[int((samples-1)*.50)],
                p95_us=durations[int((samples-1)*.95)],
                p99_us=durations[int((samples-1)*.99)], max_us=durations[-1])


# 构造显式合成阈值和消息，输出CDR大小、耗时与计算情景，配置不会安装到飞控。
def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--samples', type=int, default=5000)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    if not 1 <= args.samples <= 1000000:
        parser.error('--samples must be in 1..1000000')

    from rclpy.serialization import serialize_message
    from nav_msgs.msg import Odometry
    from jikl_fc01_system.msg import VelocityCommand, VisionHealth, VisionState
    from px4_msgs.msg import OffboardControlMode, VehicleAttitudeSetpoint, VehicleOdometry

    diagnostic = load_tool('velocity_command_diagnostic')
    adapter = load_tool('vision_odometry_adapter')
    validation = load_tool('vision_state_validation')
    command = VelocityCommand(sequence=1, source_id='benchmark-command-session',
                              frame_id='body_frd', valid=True, vx=.2, vy=-.1, vz=0.)
    command.stamp.sec = 100
    odometry = Odometry()
    odometry.header.stamp.sec = 100
    odometry.header.frame_id = 'benchmark_world'
    odometry.child_frame_id = 'benchmark_body'
    odometry.pose.pose.orientation.w = 1.
    odometry.twist.twist.linear.x = .2
    health = VisionHealth(sequence=1, source_id='benchmark-state-session',
                          frame_id='benchmark_world', body_frame_id='benchmark_body',
                          extrinsics_id='benchmark-mount', alignment_id='benchmark-alignment',
                          quality=1., angular_velocity_source='benchmark-gyro')
    health.stamp.sec = 100
    for field in ('reset_valid', 'initialized', 'tracking', 'calibration_valid',
                  'frame_alignment_valid', 'time_sync_valid', 'extrinsics_valid',
                  'metric_scale_valid', 'position_valid', 'velocity_valid',
                  'attitude_valid', 'angular_velocity_valid', 'quality_valid'):
        setattr(health, field, True)
    state = VisionState(source_id=health.source_id, frame_id='fc01_vision_ned',
                        body_frame_id='base_link_frd', angular_velocity_source='benchmark-gyro')
    heartbeat = OffboardControlMode(attitude=True)
    target = VehicleAttitudeSetpoint()
    target.q_d = [1., 0., 0., 0.]
    target.thrust_body = [0., 0., -.5]
    fixtures = dict(VelocityCommand=command, Odometry=odometry, VisionHealth=health,
                    VisionState=state, OffboardControlMode=heartbeat,
                    VehicleAttitudeSetpoint=target, VehicleOdometry=VehicleOdometry())
    sizes = {name: len(serialize_message(message)) for name, message in fixtures.items()}
    config = adapter.AdapterConfig(
        source_id=health.source_id, input_frame_id=health.frame_id,
        input_body_frame_id=health.body_frame_id, world_convention='NED', body_convention='FRD',
        body_from_sensor_wxyz=(1., 0., 0., 0.), sensor_position_body_frd=(0., 0., 0.),
        world_alignment_wxyz=(1., 0., 0., 0.), world_translation_ned=(0., 0., 0.),
        extrinsics_id=health.extrinsics_id, alignment_id=health.alignment_id)
    # Explicit synthetic policies only; none is installed as a flight setting.
    receiver = diagnostic.Receiver(diagnostic.DiagnosticConfig(
        timeout_sec=1., max_speed_mps=2., max_source_age_sec=.2,
        future_tolerance_sec=.02, expected_source_id=command.source_id))
    state_config = validation.AdapterConfig(**vars(config))
    validator = validation.Validator(validation.ValidationConfig(
        adapter=state_config, clock=validation.ClockPolicy(offset_ns=0, confirmed=True),
        receive_timeout_sec=1., max_source_age_sec=.2, future_tolerance_sec=.02,
        min_quality=.5, nominal_quality_min=.8, max_position_step_m=2.,
        max_velocity_step_mps=1., max_attitude_step_rad=.5,
        max_pose_variance=(1.,)*6, max_twist_variance=(1.,)*6))
    metadata = validation.SampleMetadata(pose_covariance_valid=True, twist_covariance_valid=True)
    for matrix in (odometry.pose.covariance, odometry.twist.covariance):
        for axis in range(6):
            matrix[axis*6+axis] = .01

    # 递增序号与纳秒时间后测一次接收诊断，立刻清事件避免队列增长混入测量。
    def receive(index):
        command.sequence = index + 1
        command.stamp.nanosec = index * 100
        event = receiver.receive(command, 10. + index * 1e-7, 100000000000 + index * 100, 1)
        if not event['accepted']:
            raise RuntimeError('benchmark command fixture rejected: ' + str(event['reasons']))
        receiver.drain_events()

    # 递增同一对测量的时戳与序号，测真实转换及原始协方差验证，拒绝则停止基准。
    def validate_state(index):
        health.sequence = index + 1
        odometry.header.stamp.nanosec = health.stamp.nanosec = index * 100
        result = validator.receive(odometry, health, 10. + index * 1e-7,
                                   100000000000 + index * 100, metadata)
        if not result.accepted:
            raise RuntimeError('benchmark state fixture rejected: ' + str(result.diagnostics))
        validator.drain_events()

    measurements = dict(
        command_serialize=measure(lambda _: serialize_message(command), args.samples),
        command_diagnostic=measure(receive, args.samples),
        odometry_conversion=measure(lambda _: adapter.convert_odometry(odometry, health, config),
                                     args.samples),
        state_validation_with_raw_covariance=measure(validate_state, args.samples))
    # 50Hz本地视觉与20Hz现有姿态双流只作流量算术；CDR不含RTPS/XRCE/串口帧及其他遥测。
    local_bytes = sum(sizes[name] for name in ('VelocityCommand', 'Odometry',
                                               'VisionHealth', 'VisionState')) * 50
    # 115200波特的8N1理论字节容量，不是可用XRCE载荷保证。
    uart_capacity = 115200 / 10  # 8 data bits + start/stop; not an XRCE payload guarantee.
    attitude_bytes = (sizes['OffboardControlMode'] + sizes['VehicleAttitudeSetpoint']) * 20
    result = dict(
        schema_version=1, host=platform.platform(), architecture=platform.machine(),
        python=sys.version, measurement_scope='host_only_no_ros_graph_or_hardware',
        cdr_bytes=sizes, measurements=measurements,
        process_peak_rss_kib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss,
        scenarios=dict(
            local_vision_at_50_hz=dict(cdr_bytes_per_second=local_bytes,
                                     added_px4_uart_bytes_per_second=0),
            existing_attitude_pair_at_20_hz=dict(cdr_bytes_per_second=attitude_bytes),
            hypothetical_px4_vehicle_odometry_at_50_hz=dict(
                implemented=False, cdr_bytes_per_second=sizes['VehicleOdometry'] * 50),
            uart_115200_8n1=dict(raw_bytes_per_second=uart_capacity)),
        limitations=[
            'No Orange Pi CPU, memory, latency or deadline validation.',
            'Peak RSS includes Python and ROS imports; it is not incremental module memory.',
            'Mean synchronous costs exclude DDS discovery, scheduling, queueing and file I/O.',
            'Per-operation percentiles include host timer overhead and are not hard realtime bounds.',
            'Command and state policy values are explicit synthetic fixtures, not flight limits.',
            'CDR sizes depend on fixture string lengths and exclude transport overhead.',
            'UART carries other telemetry; this is not a remaining-bandwidth estimate.',
            'Vision topics remain local; no PX4 EKF injection is implemented.'])
    text = json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + '\n'
    if args.output:
        args.output.write_text(text)
    print(text, end='')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
