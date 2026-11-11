#!/usr/bin/env python3
# 真实ROS本机零推力/反馈丢失冒烟：固定Domain 187，全部动作门关闭，不启动Agent或串口。
# ATTITUDE_READY只用于零推力预发送，反馈丢失后须及时静默；假sink仅证明本地提交。
"""Real ROS 2, localhost-only zero-thrust timing/feedback-loss smoke.

Run with /opt/ros/humble and the pinned px4_msgs workspace sourced.  This
process never starts an Agent or opens a serial device.  Every flight action
gate is false; the supported ATTITUDE_READY diagnostic route is stopped before
its Offboard request timer expires.  The fake sinks prove local publication
only, never PX4 adoption or flight readiness.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import tempfile
import time


# 验证已构建的真实ROS节点，生成临时关闭门配置，汇总反馈排队与丢失后的输出证据。
def main():
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--node',
        type=Path,
        default=root
        / '.build/offline-stability-20261107/install/jikl_fc01_system/lib/jikl_fc01_system/fc01_system_node',
    )
    parser.add_argument('--report', type=Path)
    parser.add_argument('--feedback-hz', type=float, default=20.0)
    parser.add_argument('--observe-seconds', type=float, default=0.0)
    args = parser.parse_args()
    if not (20.0 <= args.feedback_hz <= 50.0) or not (0.0 <= args.observe_seconds <= 180.0):
        parser.error('feedback rate must be 20..50 Hz; observation 0..180 seconds')
    if not args.node.is_file():
        parser.error('build the real ROS 2 node before this opt-in smoke')

    # Fixed isolation and fixed closed gates cannot be relaxed by CLI options.
    # 固定本机域并清除外部发现覆盖；这些隔离条件与关闭门不能通过CLI放开。
    os.environ.update(
        ROS_DOMAIN_ID='187', ROS_LOCALHOST_ONLY='1', RMW_IMPLEMENTATION='rmw_fastrtps_cpp'
    )
    for variable in (
        'ROS_DISCOVERY_SERVER',
        'CYCLONEDDS_URI',
        'FASTRTPS_DEFAULT_PROFILES_FILE',
        'FASTDDS_DEFAULT_PROFILES_FILE',
        'RMW_FASTRTPS_USE_QOS_FROM_XML',
        'ROS_STATIC_PEERS',
    ):
        os.environ.pop(variable, None)
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    import px4_msgs
    from px4_msgs.msg import (
        BatteryStatus,
        FailsafeFlags,
        ManualControlSetpoint,
        ManualControlSwitches,
        OffboardControlMode,
        VehicleAttitude,
        VehicleAttitudeSetpoint,
        VehicleCommand,
        VehicleCommandAck,
        VehicleLandDetected,
        VehicleLocalPosition,
        VehicleStatus,
    )

    # Deployment/build tooling verifies the source pin.  A filesystem path is
    # recorded below as provenance, not treated as proof of a Git commit.
    if VehicleStatus.MESSAGE_VERSION != 1 or any(
        cls.MESSAGE_VERSION != 0
        for cls in (
            VehicleLocalPosition,
            VehicleLandDetected,
            VehicleCommandAck,
            VehicleCommand,
            VehicleAttitude,
            VehicleAttitudeSetpoint,
            ManualControlSetpoint,
        )
    ):
        raise RuntimeError('incompatible px4_msgs message versions')
    rclpy.init(args=[])
    node = Node('fc01_offline_timing_smoke_source')
    process = None
    ipc = None
    records = []
    received_targets = []
    received_modes = []
    received_commands = []
    sequence = 0
    recv_buffer = b''
    publish_feedback = True
    last_publish = -1.0
    next_ping = next_status = 0.0
    sources = {}
    sensor = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
    reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
    for cls, topic in (
        (VehicleStatus, 'vehicle_status_v1'),
        (VehicleLandDetected, 'vehicle_land_detected'),
        (VehicleLocalPosition, 'vehicle_local_position'),
        (VehicleAttitude, 'vehicle_attitude'),
        (FailsafeFlags, 'failsafe_flags'),
        (BatteryStatus, 'battery_status'),
        (ManualControlSetpoint, 'manual_control_setpoint'),
        (ManualControlSwitches, 'manual_control_switches'),
        (VehicleCommandAck, 'vehicle_command_ack'),
    ):
        sources[topic] = node.create_publisher(cls, '/fmu/out/' + topic, sensor)

    # 实际收到的每个姿态目标必须是有限且精确零推力，否则立刻失败。
    def target(message):
        if any(not math.isfinite(x) or x != 0.0 for x in message.thrust_body):
            raise RuntimeError('smoke observed nonzero/invalid thrust')
        received_targets.append(message)

    # 任何VehicleCommand均违反本测试边界，收到即失败。
    def command(message):
        received_commands.append(message)
        raise RuntimeError('smoke must not emit any VehicleCommand')

    subscriptions = [
        node.create_subscription(
            VehicleAttitudeSetpoint, '/fmu/in/vehicle_attitude_setpoint', target, reliable
        ),
        node.create_subscription(
            OffboardControlMode, '/fmu/in/offboard_control_mode', received_modes.append, reliable
        ),
        node.create_subscription(VehicleCommand, '/fmu/in/vehicle_command', command, reliable),
    ]

    # 以真实ROS时钟生成已上锁Stabilized、落地静止、有效Z/VZ与RC合成反馈。
    def publish():
        stamp = node.get_clock().now().nanoseconds // 1000
        status = VehicleStatus(
            timestamp=stamp,
            system_id=1,
            component_id=1,
            arming_state=VehicleStatus.ARMING_STATE_DISARMED,
            nav_state=VehicleStatus.NAVIGATION_STATE_STAB,
            hil_state=VehicleStatus.HIL_STATE_OFF,
            pre_flight_checks_pass=True,
            power_input_valid=True,
            can_set_nav_states_mask=(1 << VehicleStatus.NAVIGATION_STATE_STAB),
        )
        land = VehicleLandDetected(
            timestamp=stamp, landed=True, at_rest=True, ground_contact=True, maybe_landed=True
        )
        local = VehicleLocalPosition(
            timestamp=stamp,
            timestamp_sample=stamp - 2000,
            z=2.0,
            xy_valid=True,
            z_valid=True,
            v_xy_valid=True,
            v_z_valid=True,
            heading_good_for_control=True,
        )
        attitude = VehicleAttitude(
            timestamp=stamp, timestamp_sample=stamp - 2000, q=[1.0, 0.0, 0.0, 0.0]
        )
        flags = FailsafeFlags(timestamp=stamp)
        battery = BatteryStatus(timestamp=stamp, id=1, connected=True, voltage_v=7.6, remaining=1.0)
        manual = ManualControlSetpoint(
            timestamp=stamp,
            timestamp_sample=stamp - 2000,
            valid=True,
            data_source=ManualControlSetpoint.SOURCE_RC,
            throttle=-1.0,
        )
        switches = ManualControlSwitches(
            timestamp=stamp,
            timestamp_sample=stamp - 2000,
            mode_slot=1,
            arm_switch=3,
            kill_switch=3,
            return_switch=3,
            loiter_switch=3,
            offboard_switch=3,
        )
        for topic, value in (
            ('vehicle_status_v1', status),
            ('vehicle_land_detected', land),
            ('vehicle_local_position', local),
            ('vehicle_attitude', attitude),
            ('failsafe_flags', flags),
            ('battery_status', battery),
            ('manual_control_setpoint', manual),
            ('manual_control_switches', switches),
        ):
            sources[topic].publish(value)

    # 通过临时IPC发送带递增序号的诊断操作，保留命令应答关联。
    def send(operation):
        nonlocal sequence
        sequence += 1
        ipc.sendall(f'{sequence} {operation}\n'.encode())
        return sequence

    # 按指定频率发布反馈、spin并维护PING/STATUS，将逐行IPC记录累积成状态证据。
    def step():
        nonlocal last_publish, ipc, recv_buffer, next_ping, next_status
        now = time.monotonic()
        if process.poll() is not None:
            raise RuntimeError('native node exited early: ' + str(process.returncode))
        if publish_feedback and now - last_publish >= 1.0 / args.feedback_hz:
            publish()
            last_publish = now
        rclpy.spin_once(node, timeout_sec=0.003)
        if ipc is None and socket_path.exists():
            ipc = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            ipc.connect(str(socket_path))
            ipc.setblocking(False)
            send('INTERACTIVE')
        if ipc is not None:
            if now >= next_ping:
                send('PING')
                next_ping = now + 0.2
            if now >= next_status:
                send('STATUS')
                next_status = now + 0.1
            while True:
                try:
                    data = ipc.recv(262144)
                except BlockingIOError:
                    break
                if not data:
                    raise RuntimeError('native IPC disconnected')
                recv_buffer += data
                while b'\n' in recv_buffer:
                    line, recv_buffer = recv_buffer.split(b'\n', 1)
                    item = json.loads(line)
                    if item.get('kind') in ('reply', 'state'):
                        records.append(item)

    # 循环执行step直到谓词成立；超时携最后状态报错，避免无期限等待。
    def until(predicate, timeout, label):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            step()
            if predicate():
                return
        raise RuntimeError(
            label + ' timed out; last state=' + json.dumps(records[-1] if records else {})
        )

    # 健康需源时钟、姿态/本地新鲜度及唯一源计数同时成立。
    def healthy(record):
        return (
            record.get('source_clock_valid')
            and record.get('attitude_fresh')
            and record.get('local_fresh')
            and record.get('source_counts') == [1, 1, 1]
        )

    with (
        tempfile.TemporaryDirectory(prefix='fc01-offline-') as temp,
        tempfile.TemporaryFile(mode='w+') as logs,
    ):
        socket_path = Path(temp) / 'control.sock'
        config_path = Path(temp) / 'closed-gates.yaml'
        params = dict(
            profile='attitude_altitude',
            socket_path=str(socket_path),
            use_sim_time=False,
            flight_enabled=False,
            flight_actions_enabled=False,
            arm_enabled=False,
            takeoff_enabled=False,
            bench_arm_only_enabled=False,
            rc_navigation_enabled=False,
            visual_navigation_enabled=False,
            flight_parameters_confirmed=False,
            rc_takeover_confirmed=False,
            autonomous_failsafe_confirmed=False,
            px4_land_fallback_confirmed=False,
        )
        config_path.write_text(
            '/**:\n  ros__parameters:\n'
            + ''.join(
                '    ' + key + ': ' + json.dumps(value) + '\n' for key, value in params.items()
            )
        )
        try:
            # Refuse pre-existing local endpoints even though the domain is fixed.
            deadline = time.monotonic() + 0.25
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.01)
            # 发现已有其他节点即拒绝实验，避免合成/fmu/out与已有本地系统混用。
            others = [name for name in node.get_node_names() if name != node.get_name()]
            if others:
                raise RuntimeError('isolated smoke domain already has nodes: ' + repr(others))
            process = subprocess.Popen(
                [str(args.node), '--ros-args', '--params-file', str(config_path)],
                stdin=subprocess.DEVNULL,
                stdout=logs,
                stderr=subprocess.STDOUT,
                env=os.environ.copy(),
            )
            until(lambda: bool(records) and healthy(records[-1]), 8.0, 'fresh real ROS callbacks')
            feedback_observation = None
            # 可选持续窗口检查RMW到回调年龄增长、停顿计数和快源多次消费，不能只检查最新一行。
            if args.observe_seconds:
                began = time.monotonic()
                begin_index = len(records)
                until(
                    lambda: time.monotonic() - began >= args.observe_seconds,
                    args.observe_seconds + 2.0,
                    'sustained feedback observation',
                )
                observed = records[begin_index:]
                window = max(1, len(observed) // 4)
                ages = {
                    name: [row['source_timing'][name]['rmw_to_callback_s'] for row in observed]
                    for name in ('local', 'attitude')
                }
                feedback_observation = dict(
                    duration_s=time.monotonic() - began,
                    requested_feedback_hz=args.feedback_hz,
                    maximum_cycle_s=max(row['max_cycle_period_s'] for row in observed),
                    stall_count=max(row['cycle_stall_count'] for row in observed),
                    sources={},
                )
                for name, values in ages.items():
                    if not values or any(value is None for value in values):
                        raise RuntimeError('RMW diagnostic unavailable for queue regression')
                    growth = sum(values[-window:]) / window - sum(values[:window]) / window
                    peak_callbacks = max(
                        row['source_timing'][name].get('callbacks_this_cycle', 1)
                        for row in observed
                    )
                    feedback_observation['sources'][name] = dict(
                        first_s=values[0],
                        last_s=values[-1],
                        max_s=max(values),
                        last_minus_first_window_s=growth,
                        max_callbacks_per_cycle=peak_callbacks,
                        max_source_age_s=max(
                            row['source_timing'][name]['envelope_age_s'] for row in observed
                        ),
                    )
                    if growth > 0.05 or max(values) > 0.15:
                        raise RuntimeError(
                            'feedback queue growth: ' + json.dumps(feedback_observation)
                        )
                    if args.feedback_hz >= 30 and args.observe_seconds >= 3 and peak_callbacks < 2:
                        raise RuntimeError(
                            'fast source was never consumed twice in a control cycle'
                        )
                if feedback_observation['stall_count'] or not all(healthy(row) for row in observed):
                    raise RuntimeError('feedback scheduling regressed safety/timing')
            fresh = records[-1]
            ready_sequence = send('ATTITUDE_READY')
            until(
                lambda: any(x.get('seq') == ready_sequence for x in records), 2.0, 'prepare reply'
            )
            ready = next(x for x in records if x.get('seq') == ready_sequence)
            if not ready['accepted']:
                raise RuntimeError('closed-gate preparation rejected: ' + ready['message'])
            until(
                lambda: len(received_targets) >= 3 and len(received_modes) >= 3,
                4.0,
                'zero-thrust prestream',
            )
            # 主动切断全部合成反馈，在Offboard请求计时到期之前验证陈旧反馈触发安全出口。
            publish_feedback = False
            stopped_at = time.monotonic()
            until(
                lambda: any(
                    x.get('phase') == 'SAFETY'
                    and not x.get('source_clock_valid')
                    and not x.get('attitude_fresh')
                    and not x.get('local_fresh')
                    for x in records
                ),
                1.5,
                'feedback-loss safety exit',
            )
            stale = records[-1]
            stale_observed_at = time.monotonic()
            # 记录静默出口后的双流数量，再观察0.2秒，确保没有继续发布的新目标或心跳。
            counts_after_exit = (len(received_targets), len(received_modes))
            until(
                lambda: time.monotonic() - stale_observed_at >= 0.2, 0.8, 'outputs remain stopped'
            )
            if counts_after_exit != (len(received_targets), len(received_modes)):
                raise RuntimeError('new outputs continued after confirmed stale SAFETY')
            prestream = next(
                x
                for x in reversed(records)
                if x.get('phase') == 'PRESTREAM' and x.get('pair_context_calls') == 3
            )
            # 每条日志都必须保持动作门关闭、无176/400提交，且PX4推力采用保持UNOBSERVED。
            for record in records:
                if any(record.get(key) for key in params if key.endswith('_enabled')):
                    raise RuntimeError('closed gate unexpectedly enabled')
                if record.get('m1_176_published') or record.get('m1_400_published'):
                    raise RuntimeError('native local audit reports a VehicleCommand')
                if record.get('px4_thrust_adoption') not in (None, 'UNOBSERVED'):
                    raise RuntimeError('simulation must not claim PX4 adoption')
            if received_commands or not subscriptions:
                raise RuntimeError('invalid fake sink evidence')
            # Status/land are still within 2.5 s, permitting the existing
            # confirmed-disarmed shutdown route without a forced termination.
            process.send_signal(signal.SIGINT)
            process.wait(timeout=3.0)
            if process.returncode != 0:
                raise RuntimeError(
                    'native process did not exit cleanly: ' + str(process.returncode)
                )
            # Drain any reliable output already in flight when SIGINT arrived,
            # so the assertion also covers shutdown rather than only the live
            # observation window.  The callbacks reject commands/nonzero thrust.
            deadline = time.monotonic() + 0.15
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.005)
            if received_commands or counts_after_exit != (
                len(received_targets),
                len(received_modes),
            ):
                raise RuntimeError('unexpected output during disarmed shutdown')
            report = dict(
                result='REAL_ROS_OFFLINE_TIMING_PASS',
                domain=187,
                localhost_only=True,
                px4_msgs_path=list(px4_msgs.__path__),
                node=str(args.node),
                closed_gate_parameters=params,
                real_target_messages=len(received_targets),
                real_heartbeat_messages=len(received_modes),
                vehicle_commands=0,
                nonzero_thrust_messages=0,
                native_exit_code=process.returncode,
                loss_to_safety_s=stale_observed_at - stopped_at,
                feedback_observation=feedback_observation,
                fresh=fresh,
                zero_prestream=prestream,
                stale=stale,
                limitation='Synthetic local ROS feedback and sinks; no Agent, UART, PX4 adoption or flight evidence.',
            )
            if args.report:
                args.report.parent.mkdir(parents=True, exist_ok=True)
                args.report.write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
            print(
                json.dumps(
                    {
                        key: value
                        for key, value in report.items()
                        if key not in ('fresh', 'zero_prestream', 'stale', 'closed_gate_parameters')
                    },
                    ensure_ascii=False,
                )
            )
        except Exception:
            logs.seek(0)
            print(logs.read()[-14000:], file=sys.stderr)
            raise
        # 这里只清理固定未解锁、零推力的本机夹具；结束子进程并关闭IPC/ROS资源。
        finally:
            if process is not None and process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=3.0)
                except subprocess.TimeoutExpired:
                    process.terminate()
                    try:
                        process.wait(timeout=3.0)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait(timeout=3.0)
            if ipc is not None:
                ipc.close()
            node.destroy_node()
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
